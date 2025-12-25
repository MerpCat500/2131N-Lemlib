/**
 * @file time_of_flight.hpp
 * @author Andrew Hilton (2131N)
 * @brief Time of flight sensor class
 * @version 0.1
 * @date 2025-11-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <random>

#include "point.hpp"
#include "pros/distance.hpp"

class DistanceSensor
{
 private:
  // Offsets relative to robot center
  Point offset;
  double heading_offset;  // in radians

  // Cache last readings to avoid redundant calculations
  double last_heading;
  Point last_position;

  // Cache trig functions for efficiency
  double cached_sin;
  double cached_cos;
  bool enabled = true;

  const double size_threshold;

  // Pointer to the field for distance calculations
  std::unique_ptr<pros::Distance> pDistance;

  // Cached distance reading
  double last_distance_reading;

  // Distance Sensor noise distribution
  const double distance_sensor_std = 25.0 / 25.4;  // 25 mm in inches
  std::normal_distribution<double> noise_dist;

 public:
  DistanceSensor(
      Point offset, double heading_offset, int distance_port, double size_threshold = 60.0)
      : offset(offset),
        heading_offset(heading_offset),
        last_heading(std::numeric_limits<double>::infinity()),
        last_position(
            {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}),
        size_threshold(size_threshold),
        cached_sin(0.0),
        cached_cos(1.0),
        pDistance(std::make_unique<pros::Distance>(distance_port)),
        last_distance_reading(0.0000001),
        noise_dist(0.0, distance_sensor_std)
  {
  }

  void set_enabled(bool enabled) { this->enabled = enabled; }

  void update(const Point& robot_position, const double& robot_heading)
  {
    bool heading_changed = (robot_heading != last_heading);
    bool position_changed = (robot_position != last_position);

    if (heading_changed)
    {
      last_heading = robot_heading;
      // Update trig values, assuming theta = 0 is on y+
      cached_sin = std::sin(M_PI_2 - (robot_heading + heading_offset));
      cached_cos = std::cos(M_PI_2 - (robot_heading + heading_offset));
    }

    // If Position changed, update the last position
    if (position_changed) { last_position = robot_position; }

    // If either changed, recalculate the distance reading
    if (heading_changed || position_changed)
    {
      Point sensor_position = get_sensor_position(robot_position, robot_heading);

      // Account for theta = 0 being y+
      last_distance_reading = pDistance->get_distance() / 25.4;  // Convert mm to inches

      if ((last_distance_reading <= 0.0 || last_distance_reading >= 143.0 * std::sqrt(2) ||
           (pDistance->get_object_size() < size_threshold && pDistance->get_object_size() != -1)) ||
          !enabled)
      {
        last_distance_reading = -1;
      }
    }
  }

  double get_cosine_cache() const { return cached_cos; }
  double get_sine_cache() const { return cached_sin; }
  Point get_offset() const { return offset; }
  Point get_sensor_position(Point robot_position, double robot_heading) const
  {
    return {
        robot_position.x + offset.x * std::sin(robot_heading) - offset.y * std::cos(robot_heading),
        robot_position.y + offset.x * std::cos(robot_heading) + offset.y * std::sin(robot_heading)};
  }

  double get_distance_reading() const { return last_distance_reading; }
  double get_distance_sensor_std() const { return distance_sensor_std; }
};