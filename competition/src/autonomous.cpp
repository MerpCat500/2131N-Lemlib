#include "autonomous.hpp"

#include <cmath>

#include "2131N/robot-config.hpp"
#include "2131N/systems/intake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"

void debug(bool is_red_team) { chassis.setPose({0, 0, 0}, false); }

void leftSideAwp(bool is_red_team)
{
  intake.setState(Intake::states::STORE_TOP);

  chassis.setPose({48 + 7.25 + 1.5, 24, -90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(24.5, 24.0, 2000, {}, false);
  chassis.turnToHeading(-180.0, 1000, {.minSpeed = 1}, false);
  chassis.moveToPoint(24.5, -100.0, 1150, {.maxSpeed = 50, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 68});
  pros::delay(1000);
  intake.setState(Intake::states::SCORING);
  pros::delay(1100);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reckon to goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24., 48 - 7.25 + 3.5, left_goal.theta});

  // ? Grab Middle
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(0., -8, 2000, {.minSpeed = 40}, false);
  chassis.moveToPose(48, 48, 10, 700, {.lead = 0.6}, false);
  chassis.moveToPose(48, 48, 10, 1000, {.lead = 0.6, .maxSpeed = 40}, false);

  // ! Score Middle
  chassis.turnToHeading(45, 1000, {}, false);
  chassis.moveToPose(60, 60, 45, 1800, {.lead = 0.2, .minSpeed = 10}, false);
  pros::delay(200);
  intake.setMiddle(true);
  intake.setState(Intake::states::SCORE_MIDDLE);
  pros::delay(1000);
  intake.setState(Intake::states::STOPPED);

  // * Go to middle blocks
  chassis.swingToHeading(135, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 30}, false);
  intake.setMiddle(false);
  intake.setState(Intake::states::STORING);
  auto pose = chassis.getPose();
  chassis.moveToRelativePose({48, -12, -45}, 700, {.lead = 0.4}, false);
  chassis.moveToPose(
      pose.x + 48, pose.y - 13, 90, 2000, {.lead = 0.4, .maxSpeed = 40, .minSpeed = 1}, false);

  // ! Score Middle Bottom
  chassis.turnToHeading(-41, 1500, {.minSpeed = 1}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(17.25, -41), 2000, {}, true);
  pros::delay(800);
  intake.setState(Intake::states::OUTTAKE);
}

void leftSide(bool is_red_team)
{
  intake.setState(Intake::states::STORING);

  chassis.setPose({48 + 7.25 + 1.5, 24, -90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(24.5, 24.0, 2000, {}, false);
  chassis.turnToHeading(-180.0, 1000, {}, false);
  chassis.moveToPoint(24.5, -100.0, 1000, {.maxSpeed = 50, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  pros::delay(2200);
  matchload_unloader.retract();
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
  chassis.moveToPoint(32, 32, 2000, {.forwards = false});
  pros::delay(400);
  intake.setMiddle(false);
  chassis.waitUntilDone();
}

void rightSide(bool is_red_team)
{
  intake.setState(Intake::states::STORING);

  chassis.setPose({-(48 + 7.25 + 1.5), 24, 90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(-24.5, 24.0, 2000, {}, false);
  chassis.turnToHeading(179.0, 1000, {.minSpeed = 1}, false);
  chassis.moveToPoint(-24.5, -100.0, 1000, {.maxSpeed = 50, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x + 0.5, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  pros::delay(2500);
  matchload_unloader.retract();
  chassis.cancelMotion();

  auto ram_goal = chassis.getPose();
  intake.setState(Intake::states::STOPPED);
  chassis.moveToPoint(ram_goal.x, ram_goal.y - 10, 1200, {.minSpeed = 1}, false);
  chassis.moveToPoint(ram_goal.x, ram_goal.y + 2, 1200, {.forwards = false, .minSpeed = 70}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(800);

  // * Middle Goal
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint({0, -10, 0}, 2000, {.minSpeed = 50}, false);
  chassis.moveToRelativePose({-24, 10, 135}, 2000, {.lead = 0.2}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(16.5, -45), 2000, {}, false);
  intake.setState(Intake::states::OUTTAKE);
  pros::delay(1000);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(36 * sqrt(2), 135), 2000, {.forwards = false}, false);
}

void skills(bool is_red_team)
{
  intake.setState(Intake::states::STORING);

  chassis.setPose({144 - (48 + 7.25 + 1.5), 24, 90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(144 - 23, 24.0, 2000, {}, false);
  chassis.turnToHeading(180, 1000, {}, false);
  chassis.moveToPoint(144 - 23, -100.0, 2800, {.maxSpeed = 40, .minSpeed = 10}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x - 0.75, after_loader.y + 36.0, 5000, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  pros::delay(5000);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto right_goal = chassis.getPose();
  chassis.setPose({144 - 24 + 1.5, 48 + 7.5, right_goal.theta});

  // !Go to loader 2
  intake.setState(Intake::states::STORING);
  chassis.moveToPose(96, 36, -90, 2000, {.lead = (0.70710678118), .minSpeed = 40}, false);
  matchload_unloader.extend();
  chassis.moveToPose(25.25, 12, 180, 5000, {.lead = 0.65}, false);

  // ! Attempt to score
  auto after_loader2 = chassis.getPose();
  chassis.moveToPoint(
      after_loader2.x - 1.2, after_loader2.y + 36.0, 5000, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  pros::delay(5000);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24, 48 + 7.5, left_goal.theta});

  // ! PARK
  chassis.moveToPose(37, 25, 100, 2000, {.lead = 0.3, .minSpeed = 60}, false);
  intake.setState(Intake::states::SCORE_MIDDLE);
  intake.setMiddle(true);
  chassis.moveToPoint(104, 14, 2000, {});
  pros::delay(200);

  matchload_unloader.extend();
  chassis.waitUntilDone();
  matchload_unloader.retract();
}

void skills2(bool is_red_team)
{
  chassis.moveToRelativePoint(Chassis::fromPolar(24, 45), 2000, {}, false);
}