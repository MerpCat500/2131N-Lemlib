/**
 * @file filters.hpp
 * @author Andrew Hilton (2131N)
 * @brief Different Mathmatical Filters
 * @version 0.1
 * @date 2025-06-16
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <sys/types.h>

#include <cmath>

class RunningAverage
{
 private:
  float previous_;  // Previous Averaged Value
  uint i;           // Iterations

 public:
  /**
   * @brief Add value to filter
   *
   * @param new_error New Value
   * @return float Adjusted Average
   */
  float filter(const float& new_error);
};

class KalmanFilter
{
 private:
  float sensor_variance_;    // Variance of Sensor
  float estimate_variance_;  // Variance of Guess
  float kalman_gain_;
  const float derivative_variance_;  // Variance of the derivative of the sensor

  float estimate_value_;  // Estimated Reality

  float last_sensor_value_ = 0.0f;       // Previous Loop Value
  float last_sensor_derivative_ = 0.0f;  // Previous Loop Change
  float last_guess_ = 0.0f;              // Last output from filter
  const float kPN_;                      // Random Noise

 public:
  /**
   * @brief Construct a new Kalman Filter
   *
   * @param sensor_variance Sensor Variance
   * @param estimate_variance Model Variance
   * @param derivative_variance Derivative of Sensor Variance
   * @param k_process_noise Tunning value for adding process noise
   */
  KalmanFilter(
      float sensor_variance,
      float estimate_variance,
      float derivative_variance,
      float k_process_noise);

  /**
   * @brief Set the Sensor Variance
   *
   * @param new_variance
   */
  void setSensorVariance(const float& new_variance);

  /**
   * @brief Filter a value
   *
   * @param measured_value Sensor reading
   * @param delta_time Time elapsed since last reading
   * @return float filtered value
   */
  float filter(const float& measured_value, const float& delta_time);

  /**
   * @brief Get the Estimate Variance
   *
   * @return float Estimated Variance
   */
  float getEstimateVariance();
  /**
   * @brief Get the Sensor Variance
   *
   * @return float Sensor Variance
   */
  float getSensorVariance();

  /**
   * @brief Get the current Kalman Gain value
   *
   * @return float Kalman Gain (0-1)
   */
  float getKalmanGain();

  /**
   * @brief Get the Estimated Value
   *
   * @return float Estimated Value
   */
  float getEstimate();
};

class ThreeTapSMA
{
 private:
  float current_value = 0;            // Current Value
  float previous_value = 0;           // Second Tap Value
  float previous_previous_value = 0;  // Third Tap Value

 public:
  /**
   * @brief Filter out new value with two previous measurements
   *
   * @param value new value
   * @return float filtered value
   */
  float filter(float value);
};