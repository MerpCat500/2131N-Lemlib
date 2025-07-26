/**
 * @file velocity_controller.hpp
 * @author Andrew Hilton (2131N)
 * @brief A Controller for managing the velocity of a motor or motor group
 * @version 0.1
 * @date 2025-06-01
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

class VelocityController
{
 private:          // Constants
  const float kS;  // Static gain
  const float kV;  // Velocity gain
  const float kA;  // Acceleration gain

  const float kP;  // Proportional gain
  const float kI;  // Integral gain
  const float kD;  // Derivative gain

  const float kSlew;  // Slew rate limit for the output

  const float integral_windup;  // Integral windup limit

  const float dead_band;  // Deadband for the controller

  float previous_error;   // Previous error for derivative calculation
  float previous_output;  // Previous output for slew calculation
  float integral = 0.0f;  // Integral term for the controller

 public:  // Methods
  /**
   * @brief Construct a new Velocity Controller object
   *
   * @param kS Static gain
   * @param kV Velocity gain
   * @param kA Acceleration gain
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param kSlew Slew rate limit for the output (voltage per second)
   * @param integral_windup Limit for integral windup
   * @param dead_band Deadband for the controller
   */
  VelocityController(
      float kS,
      float kV,
      float kA,
      float kP,
      float kI,
      float kD,
      float kSlew,
      float integral_windup,
      float dead_band);

  /**
   * @brief Calculate the output for the controller based on the current velocity and target
   * velocity
   *
   * @param current_velocity Current velocity of the motor or motor group (in x per second)
   * @param target_velocity Target velocity to achieve (in x per second)
   * @param dT Time step in milliseconds (default is 10ms)
   * @return float Output value to apply to the motor or motor group
   */
  float calculate(float current_velocity, float target_velocity, float dT = 10.0f);

  void reset();
};