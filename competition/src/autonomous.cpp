#include "autonomous.hpp"

#include <sys/_intsup.h>

#include <cmath>

#include "2131N/robot-config.hpp"
#include "2131N/systems/intake.hpp"
#include "lemlib/chassis/chassis.hpp"

void debug(bool is_red_team)
{
  chassis.setPose({0, 0, 0});
  chassis.moveToPoint(0, 24, 2000);
}

void leftSideAWP(bool is_red_team)
{
  intake.setState(Intake::states::STORING);

  chassis.setPose({48 + 7.25 + 1.5, 24, -90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(24.5, 24.0, 2000, {}, false);
  chassis.turnToHeading(-179.0, 1000, {.minSpeed = 1}, false);
  chassis.moveToPoint(24.5, -100.0, 1100, {.maxSpeed = 50, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  matchload_unloader.retract();
  pros::delay(2200);
  chassis.cancelMotion();

  auto ram_goal = chassis.getPose();
  intake.setState(Intake::states::STOPPED);
  chassis.moveToPoint(ram_goal.x, ram_goal.y - 10, 1200, {.minSpeed = 1}, false);
  chassis.moveToPoint(ram_goal.x, ram_goal.y + 2, 1200, {.forwards = false, .minSpeed = 60}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(800);
  intake.setState(Intake::states::STOPPED);

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24., 48 - 7.25 + 2.5, left_goal.theta});

  // ? Grab Middle
  intake.setState(Intake::states::SCORING);
  chassis.moveToPose(36, 36, 90, 2000, {.lead = 0.9, .minSpeed = 10}, false);
  intake.setState(Intake::states::STORING);
  chassis.moveToPose(48, 50, 10, 1800, {.lead = 0.4, .maxSpeed = 50}, false);

  // ! Score Middle
  chassis.turnToHeading(45, 1000, {}, false);
  chassis.moveToPose(61, 60, 45, 1800, {.lead = 0.2}, false);
  intake.setState(Intake::states::SCORE_MIDDLE);
  intake.setMiddle(true);
  pros::delay(1200);

  // * Retreat and wack
  intake.setState(Intake::states::OUTTAKE);
  chassis.moveToPoint(24, 24, 2000, {.forwards = false});
  pros::delay(400);
  intake.setMiddle(false);
  chassis.waitUntilDone();

  chassis.turnToHeading(180, 2000);
  chassis.moveToPoint(24, 48, 2000, {.forwards = false}, false);
}

void rightSideAWP(bool is_red_team) {
  intake.setState(Intake::states::STORING);

  chassis.setPose({-48 + 7.25 + 1.5, 24, -90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(-24.5, 24.0, 2000, {}, false);
  chassis.turnToHeading(179.0, 1000, {.minSpeed = 1}, false);
  chassis.moveToPoint(-24.5, -100.0, 1100, {.maxSpeed = 50, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  matchload_unloader.retract();
  pros::delay(2200);
  chassis.cancelMotion();

  auto ram_goal = chassis.getPose();
  intake.setState(Intake::states::STOPPED);
  chassis.moveToPoint(ram_goal.x, ram_goal.y - 10, 1200, {.minSpeed = 1}, false);
  chassis.moveToPoint(ram_goal.x, ram_goal.y + 2, 1200, {.forwards = false, .minSpeed = 60}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(800);
  intake.setState(Intake::states::STOPPED);

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({-24., 48 - 7.25 + 2.5, left_goal.theta});

  // ? Grab Middle
  intake.setState(Intake::states::SCORING);
  chassis.moveToPose(-36, 36, -90, 2000, {.lead = 0.9, .minSpeed = 10}, false);
  intake.setState(Intake::states::STORING);
  chassis.moveToPose(-48, 50, -10, 1800, {.lead = 0.4, .maxSpeed = 50}, false);

  // ! Score Middle
  chassis.turnToHeading(-45, 1000, {}, false);
  chassis.moveToPose(-61, 60, -45, 1800, {.lead = 0.2}, false);
  intake.setState(Intake::states::SCORE_MIDDLE);
  intake.setMiddle(true);
  pros::delay(1200);

  // * Retreat and wack
  intake.setState(Intake::states::OUTTAKE);
  chassis.moveToPoint(-24, 24, 2000, {.forwards = false});
  pros::delay(400);
  intake.setMiddle(false);
  chassis.waitUntilDone();

  chassis.turnToHeading(180, 2000);
  chassis.moveToPoint(-24, 48, 2000, {.forwards = false}, false);
}

void skills(bool is_red_team) {}