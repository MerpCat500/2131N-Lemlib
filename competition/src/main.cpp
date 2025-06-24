#include "main.h"

#include "pros/misc.h"
#include "robot-config.hpp"

/**
 * @brief Runs before everything else.
 *
 */
void initialize() {}

/**
 * @brief Runs when the robot is disabled.
 *
 */
void disabled() {}

/**
 * @brief Runs when field control is plugged in.
 *
 */
void competition_initialize() {}

/**
 * @brief Runs when robot is in autonomous mode.
 *
 */
void autonomous() {}

/**
 * @brief Runs when the robot is in driver control mode.
 *
 */
void opcontrol()
{
  while (true)
  {
    // Chassis tank control
    chassis.tank(
        primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
        primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y),
        true);

    // Hopper control
    if (primary.get_digital(HOPPER_BTN_IN)) { hopper.move_voltage(12000); }
    else if (primary.get_digital(HOPPER_BTN_OUT)) { hopper.move_voltage(-12000); }
    else { hopper.move_voltage(0); }

    // Bottom stage Control
    if (primary.get_digital(BTM_STAGE_BTN_IN)) { btmStage.move_voltage(12000); }
    else if (primary.get_digital(BTM_STAGE_BTN_OUT)) { btmStage.move_voltage(-12000); }
    else { btmStage.move_voltage(0); }

    // Bottom stage Control
    if (primary.get_digital(TOP_STAGE_BTN_IN)) { topStage.move_voltage(12000); }
    else if (primary.get_digital(TOP_STAGE_BTN_OUT)) { topStage.move_voltage(-12000); }
    else { topStage.move_voltage(0); }

    pros::delay(10);
  }
}