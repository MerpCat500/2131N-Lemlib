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
  mcl_localization.set_enabled(false);

  screen.addAutos({
      {"Debug", "Debug Auto, DO NOT RUN AT COMP", debug},
      {"Left Side", "Left Side Half Autonomous Win Point danielle's slay queen", leftSide},
      {"Right Side", "Right Side Half Autonomous Win Point", rightSide},
      {"Right Side Final", "Right Side Eye Candy", rightSideFinals},
      {"Left Side AWP", "Left Side Autonomous Win Point", leftSideAwp},
      {"Skills", "Skills Autonomous", skills},
      {"SafeSkills", " Safest john deer run Skills Autonomous", Safeskills},
  });

  screen.initialize(1, true);

  screen.addTelemetries(
      {{"Battery", []() { return std::to_string(pros::battery::get_capacity()); }},
       {"Position",
        []() -> std::string {
          auto position = chassis.getPose();
          return "  (" + std::to_string(position.x) + ", " + std::to_string(position.y) + ", " +
                 std::to_string(position.theta) + ")";
        }},
       {"Position (MCL)", []() -> std::string {
          auto position = mcl_localization.get_point_estimate();
          return "  X: " + std::to_string(position.x) + "  Y: " + std::to_string(position.y);
        }}});
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
void autonomous()
{
  goal_descore_left.extend();
  goal_descore_right.extend();
  middleGoalFlap.extend();

  screen.getCurrentAutoCallback()(screen.getRedTeam());
}

/**
 * @brief Runs when the robot is in driver control mode.
 *
 */
void opcontrol()
{
  intake.setIntakeMultiplier(1.0);
  intake.setMiddle(false);

  intake.setState(Intake::states::STOPPED);

  while (true)
  {
    intake.teleOp();

    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      matchload_unloader.toggle();
    }

    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
      goal_descore_left.toggle();
      goal_descore_right.extend();
    }
    else if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
      goal_descore_left.extend();
      goal_descore_right.toggle();
    }

    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) { first_stage_lift.extend(); }
    else if (primary.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_UP))
    {
      first_stage_lift.retract();
    }

    if (primary.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
      chassis.tank_with_dead_zone(
          primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 0.5,
          primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * 0.5,
          17,
          false);
    }
    else
    {
      chassis.tank_with_dead_zone(
          primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
          primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y),
          17,
          false);
    }

    pros::delay(10);
  }
}