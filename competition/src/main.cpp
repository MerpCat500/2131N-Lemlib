#include "main.h"

#include "autonomous.hpp"
#include "pros/misc.h"
#include "2131N/robot-config.hpp"

/**
 * @brief Runs before everything else.
 *
 */
void initialize()
{
  chassis.calibrate(true);

  screen.addAutos({
      {"Debug", "Debug Auto, DO NOT RUN AT COMP", debug},
  });

  screen.addTelemetries(
      {{"Battery", []() { return std::to_string(pros::battery::get_capacity()); }},
       {"Position",
        []() {
          auto position = chassis.getPose();
          return "\n  X: " + std::to_string(position.x) +
                 "\n  Y: " + std::to_string(position.y) +
                 "\n  Theta: " + std::to_string(position.theta);
        }},
       });
  
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
void autonomous() {
  screen.getCurrentAutoCallback()(screen.getRedTeam());
}

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

    pros::delay(10);
  }
}