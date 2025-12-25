/**
 * @file mcl.hpp
 * @author Andrew Hilton (2131N)
 * @brief Monte Carlo Localization Class
 * @version 0.1
 * @date 2025-11-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <array>
#include <memory>
#include <random>

#include "2131N/systems/chassis.hpp"
#include "field.hpp"
#include "particle.hpp"
#include "random.hpp"
#include "time_of_flight.hpp"

template <size_t Samples>
class Mcl
{
 private:
  // Pointer to the robot that's being localized
  Chassis* pChassis;
  std::vector<DistanceSensor*> sensors;

  Point last_chassis_position = {0, 0};
  double last_chassis_heading = 0.0;
  Point point_estimate = {0, 0};

  bool enabled = false;

  // Pointer to the field/environment
  std::shared_ptr<Field> pField;

  // Array of particles
  std::array<Particle, Samples> particles;
  std::array<double, Samples> cdf;

  // Resampling distribution
  std::uniform_real_distribution<double> resample_dist{0.0, 1.0};
  std::uniform_real_distribution<double> uniform_dist{0.0, 1.0};

  // Positional distribution
  std::normal_distribution<double> position_dist{0, 0.01};
  std::normal_distribution<double> velocity_dist{0, 0.3};
  std::normal_distribution<double> angular_velocity_dist{0, 0.3};

  // Roughening noise after resampling (very small)
  std::normal_distribution<double> roughening_dist{0.0, 0.005};

  pros::Task update_task;

  double odometry_reset_threshold = 1.0;  // Inches

 public:
  Mcl(Chassis* chassis, std::shared_ptr<Field> field, std::vector<DistanceSensor*> distance_sensors)
      : pChassis(chassis),
        last_chassis_position({chassis->getPose().x, chassis->getPose().y}),
        last_chassis_heading(chassis->getPose(true).theta),
        pField(field),
        sensors(std::move(distance_sensors)),
        update_task([this]() {
          while (true) { this->update(); }
        })
  {
    size_t index = 0;
    // Initialize particles uniformly within the environment
    size_t grid_size = static_cast<size_t>(std::sqrt(Samples));
    for (size_t i = 0; i < grid_size; i++)
    {
      for (size_t j = 0; j < grid_size; j++)
      {
        Point p{
            j * (pField->get_max_point().x / grid_size),
            i * (pField->get_max_point().y / grid_size)};

        particles[index] = Particle(p, 1.0 / static_cast<double>(Samples));

        index++;
      }
    }
  }

  Point get_point_estimate() const { return point_estimate; }

  void update()
  {
    lemlib::Pose robot_pose = pChassis->getPose(true);
    double dt = 0.0;

    while (robot_pose.x == last_chassis_position.x && robot_pose.y == last_chassis_position.y &&
           robot_pose.theta == last_chassis_heading)
    {
      robot_pose = pChassis->getPose(true);
      pros::delay(2);
      dt += 0.002;
    }

    for (auto& sensor : sensors) { sensor->update({robot_pose.x, robot_pose.y}, robot_pose.theta); }
    if (dt == 0.0)
    {
      last_chassis_position = Point{robot_pose.x, robot_pose.y};
      last_chassis_heading = robot_pose.theta;
      return;
    }

    // Initialize total weight for normalization
    double total_weight = 0.0;

    // Calculate the robot's change in movement
    Point position_delta = Point{robot_pose.x, robot_pose.y} - last_chassis_position;
    double robot_heading_delta = robot_pose.theta - last_chassis_heading;

    // Odometry-like deltas from robot motion
    const double delta_trans = std::hypot(position_delta.x, position_delta.y);
    const double delta_rot = robot_heading_delta;

    for (Particle& particle : particles)
    {
      // Sample motion in the robot's direction of travel with per-particle noise
      double noisy_trans = delta_trans + velocity_dist(rng());
      double noisy_rot = delta_rot + angular_velocity_dist(rng());

      double particle_heading = robot_pose.theta + noisy_rot;

      // Move particle along its (noisy) heading
      Point noisy_delta = {0.0, 0.0};

      noisy_delta.x +=
          noisy_trans * std::cos(M_PI_2 - particle_heading) * (1 + velocity_dist(rng()));
      noisy_delta.y +=
          noisy_trans * std::sin(M_PI_2 - particle_heading) * (1 + velocity_dist(rng()));

      particle.move(noisy_delta);

      if (particle.get_position().x < 0.0 ||
          particle.get_position().x > pField->get_max_point().x ||
          particle.get_position().y < 0.0 || particle.get_position().y > pField->get_max_point().y)
      {
        // If out of bounds, reinitialize randomly
        Point p{
            uniform_dist(rng()) * pField->get_max_point().x,
            uniform_dist(rng()) * pField->get_max_point().y};
        particle.set_position(p);
      }

      // Calculate weight based on time-of-flight sensor readings
      // For each sensor on the robot
      double exponent = 0.0;
      for (auto sensor : sensors)
      {
        // Get the sensor reading from the actual robot
        double sensor_reading = sensor->get_distance_reading();

        if (sensor_reading <= 0.0) continue;

        // Calculate the sensor position assuming the particle is at the same location
        // with the same heading as the robot
        Point particle_sensor_position =
            sensor->get_sensor_position(particle.get_position(), particle_heading);

        // Get the expected distance from the particle's sensor position to the wall
        double point_distance = pField->get_distance_to_wall(
            particle_sensor_position, sensor->get_cosine_cache(), sensor->get_sine_cache());
        exponent -= std::pow(sensor_reading - point_distance, 2.0) /
                    (2.0 * std::pow(sensor->get_distance_sensor_std(), 2.0));
      }

      if (exponent < -50) exponent = -50;  // clamp to avoid collapse
      particle.set_weight(std::exp(exponent));

      total_weight += particle.get_weight();
    }

    // Normalize weights to add up to 1
    point_estimate = Point{0.0, 0.0};

    double weight_sum_squared = 0.0;
    for (Particle& particle : particles)
    {
      particle.normalize(total_weight);

      double w = particle.get_weight();
      weight_sum_squared += w * w;

      point_estimate += particle.get_position() * w;
    }

    double n_eff = 1.0 / weight_sum_squared;

    const float resample_alpha = 0.5;
    // Only resample if effective sample size drops below threshold
    if (n_eff < Samples * resample_alpha)
    {
      // Build CDF
      cdf[0] = particles[0].get_weight();
      for (size_t i = 1; i < Samples; i++) { cdf[i] = cdf[i - 1] + particles[i].get_weight(); }

      std::array<Particle, Samples> resampled_particles;

      // Systematic resampling
      double step = 1.0 / Samples;
      double r = resample_dist(rng()) * step;  // single offset
      double u = r;

      size_t index = 0;

      for (size_t i = 0; i < Samples; i++)
      {
        // March forward through CDF
        while (u > cdf[index] && index < Samples - 1) index++;

        resampled_particles[i] = particles[index];
        resampled_particles[i].set_weight(step);

        // Roughening noise
        Point pos = resampled_particles[i].get_position();
        pos.x += roughening_dist(rng());
        pos.y += roughening_dist(rng());

        resampled_particles[i].set_position(pos);

        u += step;
      }

      // Swap back
      particles = resampled_particles;

      // Sort particles by weight.
      std::sort(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) {
        return a.get_weight() > b.get_weight();
      });

      const size_t most_likely_particles = Samples / 80;
      const size_t hill_climb_iterations = 15;

      for (size_t i = 0; i < most_likely_particles; ++i)
      {
        Particle p = particles[i];

        for (size_t j = 0; j < hill_climb_iterations; ++j)
        {
          Point current_pos = p.get_position();
          double current_weight = p.get_weight();

          // Vector of neighboring positions (small step in each direction)
          const double step_size = 0.05;
          std::array<Point, 8> neighbors = {
              {Point{current_pos.x + step_size, current_pos.y},
               Point{current_pos.x - step_size, current_pos.y},
               Point{current_pos.x, current_pos.y + step_size},
               Point{current_pos.x, current_pos.y - step_size},
               Point{current_pos.x + step_size, current_pos.y + step_size},
               Point{current_pos.x - step_size, current_pos.y - step_size},
               Point{current_pos.x + step_size, current_pos.y - step_size},
               Point{current_pos.x - step_size, current_pos.y + step_size}}};

          // For each neighbor, calculate weight
          for (const auto& neighbor_pos : neighbors)
          {
            Particle neighbor_particle = p;
            neighbor_particle.set_position(neighbor_pos);

            // Recalculate weight for neighbor
            double exponent = 0.0;
            for (auto& sensor : sensors)
            {
              double sensor_reading = sensor->get_distance_reading();
              Point particle_sensor_position =
                  sensor->get_sensor_position(neighbor_particle.get_position(), robot_pose.theta);
              double point_distance = pField->get_distance_to_wall(
                  particle_sensor_position, sensor->get_cosine_cache(), sensor->get_sine_cache());

              exponent -= std::pow(sensor_reading - point_distance, 2.0) /
                          (2.0 * std::pow(sensor->get_distance_sensor_std(), 2.0));
            }

            double neighbor_weight = std::exp(exponent) / total_weight;

            // If neighbor has higher weight, move to that position
            if (neighbor_weight > current_weight)
            {
              p.set_position(neighbor_pos);
              p.set_weight(neighbor_weight);
              current_weight = neighbor_weight;
            }
          }
        }
      }

      // Replace the most likely particles with the hill-climbed versions
      for (size_t i = 0; i < most_likely_particles; ++i) { particles[i] = particles[i]; }

      // Recalculate point estimate after resampling
      point_estimate = Point{0.0, 0.0};
      for (Particle& particle : particles)
      {
        point_estimate += particle.get_position() * particle.get_weight();
      }
    }

    auto variance = get_position_estimate_variance();
    if (variance > 14.0)
    {
      // Reinitialize all particles randomly
      for (auto& particle : particles)
      {
        Point p{
            uniform_dist(rng()) * pField->get_max_point().x,
            uniform_dist(rng()) * pField->get_max_point().y};
        particle.set_position(p);
        particle.set_weight(1.0 / static_cast<double>(Samples));
      }
    }
    else if (variance < odometry_reset_threshold && enabled)
    {
      pChassis->setPose({point_estimate.x, point_estimate.y, robot_pose.theta}, true);
    }

    last_chassis_position = Point{robot_pose.x, robot_pose.y};
    last_chassis_heading = robot_pose.theta;
  }

  // lightweight accessors for testing
  size_t get_particle_count() const { return Samples; }

  void set_odometry_reset_threshold(double threshold) { odometry_reset_threshold = threshold; }

  // return a copy of particle at index (caller should check bounds)
  Particle get_particle(size_t idx) const { return particles[idx]; }

  const std::array<Particle, Samples>& get_particles() { return particles; }

  void reset_particles(Point robot_guess, double spread)
  {
    for (auto& particle : particles)
    {
      Point p{
          robot_guess.x + uniform_dist(rng()) * spread,
          robot_guess.y + uniform_dist(rng()) * spread};
      particle.set_position(p);
      particle.set_weight(1.0 / static_cast<double>(Samples));
    }
  }

  double get_position_estimate_variance() const
  {
    // Calculate weighted variance of particle positions around the point estimate
    double variance = 0.0;

    for (const auto& particle : particles)
    {
      Point delta = particle.get_position() - point_estimate;
      double distance_squared = delta.x * delta.x + delta.y * delta.y;
      double weight = particle.get_weight();
      variance += weight * distance_squared;
    }

    return variance;
  }

  void set_enabled(bool enabled) { this->enabled = enabled; }
};