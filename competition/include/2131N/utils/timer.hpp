/**
 * @file timer.hpp
 * @author Andrew Hilton (2131N)
 * @brief Timer Class Declaration
 * @version 0.1
 * @date 2025-07-18
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <sys/types.h>

#include <cstdint>

class Timer
{
 private:
  float start_time_;     // Start time in milliseconds
  float duration_;       // Duration in milliseconds
  bool started = false;  // Flag to check if the timer has started

 public:
  /**
   * @brief Construct a new Timer
   *
   * @param duration Duration in milliseconds
   */
  Timer(float duration);

  /**
   * @brief Start the timer
   *
   * @param duration
   */
  void start(float duration = 0);

  /**
   * @brief End and reset the timer
   *
   */
  void reset();

  /**
   * @brief Get the Duration of the timer
   *
   * @return uint32_t
   */
  uint32_t getDuration() const;

  /**
   * @brief Calculate the percent of completion of timer
   *
   * @return float Percentage Complete
   */
  float getPercentComplete();
};