#include "2131N/utils/timer.hpp"

#include "pros/rtos.hpp"

Timer::Timer(float duration) : start_time_(pros::millis()), duration_(duration) {}

void Timer::start(float duration)
{
  if (duration == 0)  // If no duration is specified, use the current duration
  {
    duration = duration_;
  }

  // Reset start time and duration
  start_time_ = pros::millis();
  duration_ = duration;

  started = true;  // Mark as started
}

void Timer::reset()
{
  started = false;  // No longer started
}

uint32_t Timer::getDuration() const { return duration_; }

float Timer::getPercentComplete()
{
  // If started then return the percentage of the way complete
  if (started) { return (pros::millis() - start_time_) / duration_; };

  // Else 0% completion
  return 0.0;
}