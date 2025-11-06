#include "main.h"

#include "2131N/robot-config.hpp"
#include "autonomous.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

/**
 * @brief Runs before everything else.
 *
 */
void initialize()
{
  chassis.calibrate(true);

  screen.addAutos(
      {{"Debug", "Debug Auto, DO NOT RUN AT COMP", debug},
       {"Left Side", "Left Side Half Autonomous Win Point", leftSide},
       {"Right Side", "Right Side Half Autonomous Win Point", rightSide},
       {"Left Side AWP", "Left Side Autonomous Win Point", leftSideAwp},
       {"Skills", "Skills Autonomous", skills},
       {"Skills Two", "A Higher Scoring Auto... maybe :/", skills2}});

  screen.initialize(5, true);

  screen.addTelemetries({
      {"Battery", []() { return std::to_string(pros::battery::get_capacity()); }},
      {"Position",
       []() {
         auto position = chassis.getPose();
         return "\n  X: " + std::to_string(position.x) + "\n  Y: " + std::to_string(position.y) +
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
void autonomous() { screen.getCurrentAutoCallback()(screen.getRedTeam()); }

/**
 * @brief Runs when the robot is in driver control mode.
 *
 */
void opcontrol()
{
  intake.setState(Intake::states::STOPPED);

  while (true)
  {
    intake.teleOp();

    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      matchload_unloader.toggle();
    }

    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
      pros::Task(
          []() {
            goal_descore.extend();
            pros::delay(200);
            goal_descore.retract();
          },
          "Goal Descore Task");
    }

    if (primary.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
      chassis.tank(
          primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 0.5,
          primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * 0.5,
          true);
    }
    else
    {
      chassis.tank(
          primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
          primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y),
          true);
    }

    pros::delay(10);
  }
}