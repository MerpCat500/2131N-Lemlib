/**
 * @file intake.hpp
 * @author Andrew Hilton (2131N)
 * @brief Declaration of the Intake Class
 * @version 0.1
 * @date 2025-07-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <vector>

#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"

class Intake
{
 public:  // State System
  enum class IntakeState
  {
    IGNORE,        // Used to IGNORE intake state
    IDLE,          // Turns off intake (ie. brake)
    INTAKE,        // Turns on the intake
    SCORE_MIDDLE,  // Sets intake so it's trying to score in the middle goal
    OUTTAKE,       // Spins intake in reverse as to outtake
  };

  enum class StorageState
  {
    IGNORE,   // used to IGNORE storage state
    IDLE,     // Turns off the storage (ie. brake)
    STORE,    // Turns on the storage to store intaking balls
    UNSTORE,  // Spins the storage in reverse as to release all the balls
  };

 private:
  pros::Motor* bottom_stage_;  // Pointer to the bottom stage motor
  pros::Motor* top_stage_;     // Pointer to the top stage motor
  pros::Motor* storage_;       // Pointer to the storage motor

  std::vector<pros::Optical*> color_sensors_;  // list of pointers to color sensors

  pros::Controller* primary_;  // Controller (for tele-op)

  pros::controller_digital_e_t
      intake_button_;  // Intake Button (Spins Top and Bottom Stage to cycle ball up)
  pros::controller_digital_e_t
      outtake_button_;  // Outtake Button (Spins Top and Bottom Stage to cycle ball down)

  pros::controller_digital_e_t
      store_button_;  // Store Button (Spins Storage in to store intaking balls)
  pros::controller_digital_e_t
      unstore_button_;  // Unstore Button (Spins Storage out to remove stored balls)

  pros::Task color_sort_task_;       // Color Sort task
  bool color_sort_enabled_ = false;  // Whether or not Color Sort is enabled

  const float
      red_sort_bound_;  // Boundary at which to sort out red ball (separates from background noise)
  const float blue_sort_bound_;  // Boundary at which to sort out blue ball

  bool is_red_team_ = true;  // Are we on red or blue team?

  bool sorting_ = false;  // Is the Sort Task Sorting a ball?

  // Pre Sort States and Current States
  IntakeState pre_sort_intake_state_ = IntakeState::IDLE;
  IntakeState current_intake_state_ = IntakeState::IDLE;

  StorageState pre_sort_storage_state_ = StorageState::IDLE;
  StorageState current_storage_state_ = StorageState::IDLE;

 public:
  /**
   * @brief Construct an Intake
   *
   * @param top_stage_motor Pointer to top stage motor
   * @param bottom_stage_motor Pointer to bottom stage motor
   * @param storage_motor Pointer to storage motor
   * @param color_sensors List of pointers to color sensors
   * @param controller Pointer to the controller
   * @param intake_button Intake Button
   * @param outtake_button Outtake Button
   * @param store_button Store Button
   * @param unstore_button Unstore Button
   * @param red_sort_bound Red Sorting Bound
   * @param blue_sort_bound Blue Sorting Bound
   */
  Intake(
      pros::Motor* top_stage_motor,
      pros::Motor* bottom_stage_motor,
      pros::Motor* storage_motor,
      std::vector<pros::Optical*> color_sensors,
      pros::Controller* controller,
      pros::controller_digital_e_t intake_button,
      pros::controller_digital_e_t outtake_button,
      pros::controller_digital_e_t store_button,
      pros::controller_digital_e_t unstore_button,
      const float red_sort_bound,
      const float blue_sort_bound);

  void teleOp();

  void setState(
      IntakeState intake_state = IntakeState::IGNORE,
      StorageState storage_state = StorageState::IGNORE,
      std::int16_t voltage = 12000);

  void setSortEnabled(bool sort_enabled, bool is_red_team);

 private:
  void update();
};