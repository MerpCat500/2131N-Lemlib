#include "main.h"

#include "2131N/robot-config.hpp"
#include "autonomous.hpp"
#include "pros/misc.h"

/**
 * @brief Runs before everything else.
 *
 */
void initialize()
{
  chassis.calibrate(true);

  screen.addAutos({
      {"Debug", "Debug Auto, DO NOT RUN AT COMP", debug},
      {"Left Side", "Left Side Half Autonomous Win Point", leftSideAWP},
      {"Right Side", "Right Side Half Autonomous Win Point", rightSideAWP},
      {"Skills", "Skills Autonomous", skills},

  });

  screen.addTelemetries({
      {"Battery", []() { return std::to_string(pros::battery::get_capacity()); }},
      {"Position",
       []() {
         auto position = chassis.getPose();
         return "\n  X: " + std::to_string(position.x) + "\n  Y: " + std::to_string(position.y) +
                "\n  Theta: " + std::to_string(position.theta);
       }},
  });

  screen.initialize(2, true);
}

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
void autonomous() { screen.getCurrentAutoCallback()(screen.getRedTeam()); }

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

    intake.teleOp();

    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      matchload_unloader.toggle();
    }

    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { storage_unstore.toggle(); }

    pros::delay(10);
  }
}