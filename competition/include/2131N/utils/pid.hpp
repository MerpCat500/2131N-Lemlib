/**
 * @file pid.hpp
 * @author Andrew Hilton (2131N)
 * @brief PID Controller Declaration
 * @version 0.1
 * @date 2025-06-20
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

class PID
{
 private:          // Constants
  const float kP;  // Proportional gain
  const float kI;  // Integral gain
  const float kD;  // Derivative gain

  const float integral_windup;  // Integral windup limit

  float previous_error;   // Previous error for derivative calculation
  float integral = 0.0f;  // Integral term for the controller

 public:  // Methods
  /**
   * @brief Construct a new Velocity Controller object
   *
   * @param kP Proportional gain
   * @param kI Integral gain
   * @param kD Derivative gain
   * @param integral_windup Limit for integral windup
   */
  PID(float kP, float kI, float kD, float integral_windup = 0.0);

  /**
   * @brief Calculate the output for the controller based on the current velocity and target
   * velocity
   *
   * @param error (target value - actual value)
   * @param dT Time step in milliseconds (default is 10ms)
   * @return float Output value to apply to the motor or motor group
   */
  float calculate(float error, float dT = 10.0f);
};