#include "2131N/utils/velocity_controller.hpp"

#include <cmath>

#include "2131N/utils/math.hpp"

VelocityController::VelocityController(
    float kS,
    float kV,
    float kA,
    float kP,
    float kI,
    float kD,
    float kSlew,
    float integral_windup,
    float dead_band)
    : kS(kS),
      kV(kV),
      kA(kA),
      kP(kP),
      kI(kI),
      kD(kD),
      kSlew(kSlew),
      integral_windup(integral_windup),
      previous_error(0.0f),
      previous_output(0.0f),
      dead_band(dead_band)
{
}

float VelocityController::calculate(float current_velocity, float target_velocity, float dT)
{
  // Calculate the estimated voltage to apply to the motor based on the target velocity
  float feedforward = kS * sign(target_velocity) + kV * target_velocity +
                      kA * (target_velocity - current_velocity) / dT;

  // Calculate the feedback control terms
  float error = target_velocity - current_velocity;  // Where we are vs where we want to be

  if (fabs(target_velocity) < dead_band)
  {
    // If the error is within the deadband, return 0 to avoid unnecessary movement
    previous_error = error;  // Update previous error to maintain integral and derivative terms
    integral = 0.0f;         // Reset integral to avoid windup
    return 0.0f;
  }

  float proportional = kP * error;  // Proportional term

  // Sum up the integral term, clamping it to prevent windup
  // Use the average of the current and previous error to do a better Riemann sum
  if (fabs(error) < integral_windup || integral_windup == 0.0)
  {
    integral += kI * (error + previous_error) / 2.0f * dT;
  }
  else { integral = 0.0f; }

  // Derivative term, using the change in error over time
  float derivative = kD * (error - previous_error) / dT;

  // Update the previous error for the next iteration
  previous_error = error;

  // Return the total output, which is the sum of feedforward and feedback terms
  float output = feedforward + proportional + integral + derivative;

  float voltage_derivative = (output - previous_output);

  // Apply a slew rate limit to the output
  if (std::fabs(voltage_derivative) > kSlew)
  {
    output = previous_output + std::copysign(kSlew, voltage_derivative) * dT;
  }

  previous_output = output;  // Store the output for future use

  return output;
}

void VelocityController::reset()
{
  previous_error = 0.0f;  // Update previous error to maintain integral and derivative terms
  integral = 0.0f;        // Reset integral to avoid windup
}