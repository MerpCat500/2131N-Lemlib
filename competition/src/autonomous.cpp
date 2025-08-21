#include "autonomous.hpp"

#include <sys/_intsup.h>

#include <cmath>

#include "2131N/robot-config.hpp"
#include "2131N/systems/intake.hpp"
#include "lemlib/chassis/chassis.hpp"

void debug(bool is_red_team)
{
  chassis.setPose({0, 0, 0});
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
}

void leftSideAWP(bool is_red_team)
{
  // ! InitialS
  chassis.setPose({48.0 + 7.0f, 24 - 7 - 9, 0});

  // * Intake Group of three
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPose(48.0, 48, -30, 2000, {.lead = 0.2}, false);

  // * Score in middle goals
  chassis.turnToPoint(72.0, 72, 800, {}, false);
  chassis.moveToPose(72 - 12.7 - 2, 72 - 12.7 - 2, 45, 1000, {.lead = 0.1}, false);
  intake.setState(Intake::IntakeState::SCORE_MIDDLE, Intake::StorageState::UNSTORE, 10000);
  pros::delay(1000);
  intake.setState(Intake::IntakeState::SCORE_MIDDLE, Intake::StorageState::UNSTORE, 12000);
  pros::delay(1000);

  // ? Go to Matchloader
  chassis.moveToPoint(23.5, 24, 2200, {.forwards = false}, false);
  chassis.turnToHeading(180, 800, {}, false);

  // * Matchloader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for piston to extend
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPoint(23.5, 0, 950, {.maxSpeed = 66, .minSpeed = 66}, false);

  // * Back Up
  auto pose1 = chassis.getPose();
  chassis.moveToPoint(pose1.x - 2, pose1.y + 10, 1000, {.forwards = false}, true);

  pros::delay(300);
  matchload_unloader.retract();
  pros::delay(200);
  chassis.turnToHeading(0, 1200, {.maxSpeed = 80});

  auto pose2 = chassis.getPose();
  chassis.moveToPose(pose2.x - 0.5, pose2.y + 20, 0, 1200, {.lead = 0.2}, false);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::UNSTORE);

  // // * Clear the matchload tube
  // chassis.swingToHeading(-70, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 80}, false);
  // intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  // matchload_unloader.extend();
  // chassis.moveToPoint(pose2.x, pose2.y - 100, 1200, {.maxSpeed = 60, .minSpeed = 60}, false);

  chassis.waitUntilDone();
}

void rightSideAWP(bool is_red_team)
{
  // ! Initial
  chassis.setPose({-(48.0f + 7.0f), 24 - 7 - 9, -0});

  // * Intake Group of three
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPose(-49.0, 48, 45, 2000, {.lead = 0.2}, false);

  // * Score in middle goals
  chassis.turnToPoint(-72.0, 72, 800, {}, false);
  chassis.moveToPose(-(72.5 - 12.7 - 1), 71.5 - 12.7 - 1, -43, 1000, {.lead = 0.1}, false);
  intake.setState(Intake::IntakeState::OUTTAKE, Intake::StorageState::UNSTORE, 6000);
  pros::delay(2000);

  // ? Go to Matchloader
  chassis.moveToPoint(-25, 24, 3000, {.forwards = false, .maxSpeed = 100}, false);
  chassis.turnToHeading(-180, 800, {}, false);

  // * Matchloader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for piston to extend
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPoint(-24, -1000, 900, {.maxSpeed = 60, .minSpeed = 60}, false);

  // * Back Up
  auto pose1 = chassis.getPose();
  chassis.moveToPoint(pose1.x - 1.5, pose1.y + 10, 1000, {.forwards = false}, true);

  pros::delay(300);
  matchload_unloader.retract();
  pros::delay(200);
  chassis.turnToHeading(-0, 1200, {.maxSpeed = 80});

  auto pose2 = chassis.getPose();

  chassis.moveToPose(pose2.x - 0.25, pose2.y + 19, -0, 1200, {.lead = 0.2}, false);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::UNSTORE);

  // // * Clear the matchload tube
  // chassis.swingToHeading(70, lemlib::DriveSide::RIGHT, 2000, {.minSpeed = 80}, false);
  // intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  // matchload_unloader.extend();
  // chassis.moveToPoint(pose2.x, pose2.y - 100, 1200, {.maxSpeed = 60, .minSpeed = 60}, false);

  chassis.waitUntilDone();
}

void skills(bool is_red_team)
{
  // ! Initial
  chassis.setPose({72 + 7, 25.5, 90});

  // * Go to loader
  chassis.moveToPoint(120, 25.5, 2000, {}, false);
  chassis.turnToPoint(121.5, 0, 1000, {}, false);

  float unscore_speed = 100;

  // * Unload Loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for piston to extend
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPoint(
      121.5, -1000, 2800, {.maxSpeed = unscore_speed, .minSpeed = unscore_speed}, false);

  // * Back Up
  auto pose1 = chassis.getPose();
  chassis.moveToPoint(pose1.x - 1.5, pose1.y + 10, 1000, {.forwards = false}, true);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::IDLE);
  pros::delay(300);
  matchload_unloader.retract();
  pros::delay(200);
  chassis.waitUntilDone();

  // * Score balls
  chassis.turnToHeading(-0, 1200, {.maxSpeed = 80});
  chassis.moveToPose(pose1.x - 1.5, pose1.y + 30, 0, 1200, {.lead = 0.2, .minSpeed = 20}, false);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::UNSTORE);
  pros::delay(3000);

  // * Leaving
  auto pose2 = chassis.getPose();
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPoint(pose2.x, pose2.y - 10, 2000, {.forwards = false}, false);
  chassis.turnToPoint(pose2.x - 18, pose2.y - 10, 2000, {.minSpeed = 1}, false);
  chassis.moveToPose(pose2.x - 18, pose2.y - 10, -90, 2000, {.lead = 0.2}, false);

  pros::delay(100);

  // ! Wall Reset
  chassis.setPose(
      {144.0f - (((back_distance.get_distance() / 25.4f + 7.5f) *
                  std::cos(-(chassis.getPose(true).theta + static_cast<float>(M_PI_2))))),
       (left_distance.get_distance() / 25.4f + 5.75f),
       chassis.getPose().theta});

  unscore_speed = 100;

  // * Move To Loader 2
  chassis.moveToPoint(106, 120, 4000, {}, false);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.turnToHeading(-90, 1600, {}, false);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::UNSTORE);

  // ! Wall Reset
  chassis.setPose(
      {144.0f - ((back_distance.get_distance() / 25.4f + 7.5f) *
                 std::cos(-(chassis.getPose(true).theta + static_cast<float>(M_PI_2)))),
       chassis.getPose().y,
       chassis.getPose().theta});

  chassis.moveToPoint(120, 120, 2000, {.forwards = false}, false);

  // * Unload Loader 2
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  matchload_unloader.extend();
  pros::delay(200);  // Wait for piston to extend
  chassis.turnToHeading(0, 800, {}, false);
  chassis.moveToPoint(
      120, 20000, 2000, {.maxSpeed = unscore_speed, .minSpeed = unscore_speed}, false);

  // * Back Up
  auto pose3 = chassis.getPose();
  chassis.moveToPoint(pose3.x, pose3.y - 10, 1000, {.forwards = false}, true);

  pros::delay(300);
  matchload_unloader.retract();
  pros::delay(200);
  chassis.waitUntilDone();

  // * Score balls
  chassis.turnToHeading(180, 1200, {.maxSpeed = 80});
  chassis.moveToPose(pose3.x + 1.25, pose3.y - 26.5, -180, 1200, {.lead = 0.2}, false);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::UNSTORE);
  pros::delay(4000);  // Let Balls Score

  // ! Tare Position
  chassis.setPose(
      {144.0f - (left_distance.get_distance() / 25.4f + 5.75f),
       96 + 7 + 2,
       chassis.getPose().theta});

  unscore_speed = 100;

  // * Loader 3
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPoint(24, 120, 4000, {.forwards = false}, false);
  matchload_unloader.extend();
  pros::delay(200);  // Wait for piston to extend
  chassis.turnToHeading(0, 800, {}, false);
  chassis.moveToPoint(
      24, 2000, 2200, {.maxSpeed = unscore_speed, .minSpeed = unscore_speed}, false);

  // * Back Up
  auto pose4 = chassis.getPose();
  chassis.moveToPoint(pose4.x, pose4.y - 10, 1000, {.forwards = false}, true);

  pros::delay(300);
  matchload_unloader.retract();
  pros::delay(200);
  chassis.waitUntilDone();

  // * Score balls
  chassis.turnToHeading(180, 1200, {.maxSpeed = 80});
  chassis.moveToPose(pose4.x + 1, pose4.y - 27, -180, 1200, {.lead = 0.2}, false);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::UNSTORE);
  pros::delay(5000);  // Let Balls Score

  // * Retreat
  chassis.moveToPoint(pose4.x, pose4.y - 10, 1500, {.forwards = false}, false);
  chassis.turnToPoint(pose4.x + 24, pose4.y - 10, 800, {}, false);
  chassis.moveToPoint(pose4.x + 24, pose4.y - 10, 2000, {}, false);

  // ! Wall Reset
  chassis.setPose(
      {back_distance.get_distance() / 25.4f + 7.5f,
       144.0f - (left_distance.get_distance() / 25.4f + 5.75f),
       chassis.getPose().theta});

  // * Go To Loader 4
  intake.setState(Intake::IntakeState::OUTTAKE, Intake::StorageState::STORE);
  chassis.moveToPoint(48, 24, 4000, {}, false);
  chassis.turnToHeading(90, 2000, {}, false);

  // ! Wall Reset
  chassis.setPose(
      {back_distance.get_distance() / 25.4f + 7.5f, chassis.getPose().y, chassis.getPose().theta});

  // * Stage Loader 4 Clear
  chassis.moveToPoint(24, 24, 4000, {.forwards = false}, false);
  matchload_unloader.extend();
  chassis.turnToHeading(-180, 800, {}, false);

  // * Loader 4 Clear
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPoint(
      24, -2000, 2000, {.maxSpeed = unscore_speed, .minSpeed = unscore_speed}, false);

  // * Back Up
  auto pose5 = chassis.getPose();
  chassis.moveToPoint(pose5.x - 1, pose5.y + 10, 1000, {.forwards = false}, true);

  pros::delay(300);
  matchload_unloader.retract();
  pros::delay(200);
  chassis.waitUntilDone();

  // * Score balls
  chassis.turnToHeading(0, 1200, {.maxSpeed = 80});
  chassis.moveToPose(pose5.x - 1, pose5.y + 27, 0, 1200, {.lead = 0.2}, false);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::UNSTORE);
  pros::delay(5000);  // Let Balls Score
}