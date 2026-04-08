#include "main.h"

#include "2131N/robot-config.hpp"
#include "autonomous.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
  

//int topIntakeSpeed;
//bool scoreSpeedPressed = false;

/**
 * @brief Runs before everything else.
 *
 */

void initialize()
{
  chassis.calibrate(true);
  mcl_localization.set_enabled(false);

  screen.addAutos({
      {"Debug", "Debug Auto, DO NOT RUN AT COMP", debug},     //this one counts as 0, so left side is 1
      {"Left Side", "Left Side Half Autonomous Win Point danielle's slay queen", leftSide},  //1
      {"Right Side", "Right Side Half Autonomous Win Point", rightSide},  //2
      {"MOve ONe Inch", "Right Side Eye Candy", rightSideFinals},  //3
      {"RIGHT Side AWP ♥", "Right Side SOLOOOOOOOO Autonomous Win Point ♥", leftSideAwp}, //4
      {"Skills", "Skills Autonomous", skills}, //5
      {"SafeSkills", " Safest john deer run Skills Autonomous", Safeskills}, //6
      {"VistaSkills", " vista half feild style", VistaSkills}, //7
      {"Left Side 7 Block", " Left Side 7 Block", LeftSide7Block},  //8
      {"Right Side 7 Block", " Right Side 7 Block ", RightSide7Block}, //9
      {"Right Side Double Middle", " Rigt side 2x middle ", RightSideDoubleMiddle}, //10
      {"Right Side 9 Block", " Right Side 9 Block ", RightSide9Block}, //11
  }); 

  screen.initialize(10, true);

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
  //middle_lift.extend();
  goal_descore_right.extend();
  //middleGoalFlap.extend();

  screen.getCurrentAutoCallback()(screen.getRedTeam());
}

/**
 * @brief Runs when the robot is in driver control mode.
 *
 */
void opcontrol()
{
  intake.setIntakeMultiplier(1.0, 1.0, 1.0);
  intake.setMiddle(false);

  intake.setState(Intake::states::STORING);
  intake.setState(Intake::states::STOPPED);

  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

  while (true)
  {
    intake.teleOp();

    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
      matchload_unloader.toggle();
    }

    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2))
    {
      middle_lift.toggle();
      // goal_descore_right.extend();
    }
    else if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
      goal_descore_right.toggle();
    }
    // else if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
    // {
    // storage_block.toggle();
    // }

    // if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
    // { 
    //   //first_stage_lift.extend();
    // }
    // else if (primary.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_UP))
    // {
    //   //first_stage_lift.retract();
    // }
    if (primary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {

      matchload_unloader.retract();
      pros::delay(0);
      middle_descore.extend();

    }
    else if(primary.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_B))
    {
      middle_descore.retract();
    }

    if (primary.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
      chassis.tank_with_dead_zone(
          primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) * 0.5,
          primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) * 0.5,
          17,
          false);
          
          intake.setIntakeMultiplier(1.0, 1.0, 0.29);
      
    } 
    
    else
    {
      chassis.tank_with_dead_zone(
          primary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
          primary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y),
          17,
          false);
      
          intake.setIntakeMultiplier(1.0, 1.0, 1.0);
    }
  }
}