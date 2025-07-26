/**
 * @file filters.cpp
 * @author Andrew Hilton (2131N)
 * @brief Mathmatical Filter Definitions
 * @version 0.1
 * @date 2025-06-20
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "2131N/utils/filters.hpp"

float RunningAverage::filter(const float& newError)
{
  i++;  // Increment the iterations

  // Recalculate the running average
  previous_ = previous_ + 1.0f / i * (newError - previous_);

  // Return new Value
  return previous_;
}

KalmanFilter::KalmanFilter(
    float sensor_variance,
    float estimate_variance,
    float derivative_variance,
    float k_process_noise)
    : sensor_variance_(sensor_variance),
      estimate_variance_(estimate_variance),
      derivative_variance_(derivative_variance),
      kPN_(k_process_noise)
{
}

void KalmanFilter::setSensorVariance(const float& newVariance) { sensor_variance_ = newVariance; }

float KalmanFilter::filter(const float& measured_value, const float& delta_time)
{
  if (delta_time == 0.0f) { return last_guess_; }

  // Add a little bit of randomness to account for process noise
  estimate_variance_ += kPN_ + (derivative_variance_ * delta_time);

  // Calculate Kalman Gain (Minimizes Variance of Estimation)
  kalman_gain_ = estimate_variance_ / (estimate_variance_ + sensor_variance_);

  // Update Estimate Variance
  estimate_variance_ *= (1.0f - kalman_gain_);

  // Calculate Estimate
  estimate_value_ = last_guess_;  //+ last_sensor_derivative_ * delta_time;

  last_guess_ = measured_value * kalman_gain_ + estimate_value_ * (1 - kalman_gain_);

  last_sensor_derivative_ = (measured_value - last_sensor_value_) / delta_time;
  last_sensor_value_ = measured_value;
  return last_guess_;
}

float KalmanFilter::getEstimateVariance() { return estimate_variance_; }
float KalmanFilter::getSensorVariance() { return sensor_variance_; }
float KalmanFilter::getKalmanGain() { return kalman_gain_; }
float KalmanFilter::getEstimate() { return estimate_value_; }

float ThreeTapSMA::filter(float value)
{
  // Update Values
  previous_previous_value = previous_value;
  previous_value = current_value;
  current_value = value;

  // Return average of all the values
  return (current_value + previous_value + previous_previous_value) / 3.0f;
}