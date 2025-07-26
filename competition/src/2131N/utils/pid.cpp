#include "2131N/utils/pid.hpp"

#include <cmath>

PID::PID(float kP, float kI, float kD, float integral_windup)
    : kP(kP), kI(kI), kD(kD), integral_windup(integral_windup), previous_error(0.0f)
{
}

float PID::calculate(float error, float dT)
{
  // Calculate the feedback control terms
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
  return proportional + integral + derivative;
}