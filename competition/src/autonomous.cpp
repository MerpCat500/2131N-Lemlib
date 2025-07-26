#include "autonomous.hpp"

#include <sys/_intsup.h>

#include "2131N/robot-config.hpp"
#include "2131N/systems/intake.hpp"

void debug(bool is_red_team)
{
  chassis.setPose({0, 0, 0});
  chassis.moveToPoint(0, 24, 2000, {}, false);
}

void leftSideAWP(bool is_red_team)
{
  // ! Initial
  chassis.setPose({72 - 9 - 7 - 0.5 - 2, 25 - 13.5 / 2, -90});

  // * Move to Loader
  chassis.moveToPoint(24.5, 25 - 13.5 / 2 + 2, 2000, {}, false);
  chassis.turnToPoint(24.5, 0, 800, {}, false);

  // * Unload Matchload
  matchload_unloader.extend();
  pros::delay(500);  // Delay to let Piston Extend
  chassis.moveToPoint(24.5, 0, 1400, {.minSpeed = 127}, true);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);

  pros::delay(650);              // Delay for balls
  matchload_unloader.retract();  // Retract Piston
  chassis.waitUntilDone();       // Wait Until Moving forwards is done

  //! Reset Pose
  auto current_pose = chassis.getPose();
  chassis.setPose(
      {current_pose.x , 14 - 9.7 - 3, current_pose.theta});  // Cancel out wheel drift

  chassis.moveToPoint(24, 24, 800, {.forwards = false, .maxSpeed = 80}, false);
  pros::delay(200);

  intake.setState(Intake::IntakeState::IDLE, Intake::StorageState::STORE);

  chassis.turnToPoint(22, 48, 1000, {}, false);
  chassis.moveToPoint(24, 34, 800, {.minSpeed = 20}, false);

  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::UNSTORE);
}