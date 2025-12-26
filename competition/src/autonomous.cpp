#include "autonomous.hpp"

#include <cmath>

#include "2131N/robot-config.hpp"
#include "2131N/systems/chassis.hpp"
#include "2131N/systems/intake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"

void debug(bool is_red_team) { chassis.setPose({0, 0, 0}, false); }

void leftSideAwp(bool is_red_team)
{
  intake.setState(Intake::states::STORE_TOP);
  chassis.setPose({49 + 5.75 + 1, 24, -90}, false);

  // UNLOAD GOAL 1
  matchload_unloader.extend();
  chassis.moveToPoint(24, 24, 2000, {}, false);

  // Grab Blocks
  chassis.turnToHeading(-180, 1000, {.minSpeed = 1}, false);
  chassis.moveToPoint(24, -100.0, 1200, {.maxSpeed = 80, .minSpeed = 30}, false);

  // SCORE GOAL 1
  chassis.moveToRelativePoint(0.0, 38, 1500, {.forwards = false, .maxSpeed = 100}, true);
  pros::delay(500);
  intake.setState(Intake::states::SCORING);
  pros::delay(1000);
  chassis.waitUntilDone();
  matchload_unloader.retract();

  // ! RESET TO LEFT GOAL 1
  auto left_goal_1 = chassis.getPose();
  chassis.setPose(23.5, 40.5, left_goal_1.theta);

  // GO TO GOAL 2
  chassis.moveToRelativePoint(0, -6, 1000, {.minSpeed = 10}, true);

  chassis.turnToHeading(90, 1000, {.minSpeed = 1}, false);

  // Go to left middle cluster
  chassis.moveToPose(41, 44, 45, 2000, {.lead = 0.3, .minSpeed = 30}, false);
  intake.setState(Intake::states::STORING);

  // Pick up blocks
  chassis.moveToRelativePoint(
      Chassis::fromPolar(6 * sqrt(2), 45), 3000, {.maxSpeed = 50, .minSpeed = 50}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(16 * sqrt(2), 45), 1000, {}, true);
  pros::delay(800);
  intake.setState(Intake::states::SCORE_MIDDLE);

  // Score in goal
  chassis.waitUntilDone();
  intake.setMiddle(true);
  pros::delay(1000);
  intake.setState(Intake::states::STORING);

  // Retreat to next cluster
  chassis.moveToPoint(48, 52, 2000, {.forwards = false, .minSpeed = 10}, false);
  intake.setMiddle(false);

  // Go to right middle cluster
  chassis.turnToPoint(96, 52, 1500, {.minSpeed = 10}, false);

  chassis.moveToPoint(96, 52, 500, {.maxSpeed = 120, .minSpeed = 20}, false);
  chassis.moveToPoint(96, 52, 1000, {.maxSpeed = 40, .minSpeed = 20}, false);

  // Final Loader
  chassis.turnToPoint(117, 36, 1500, {.minSpeed = 10}, false);
  chassis.moveToPoint(117, 36, 2000, {.maxSpeed = 100, .minSpeed = 20}, false);

  chassis.turnToHeading(180, 1200, {}, false);
  chassis.moveToRelativePoint(0.0, 38, 1800, {.forwards = false, .maxSpeed = 100}, true);
  pros::delay(300);
  intake.setState(Intake::states::SCORING);
  pros::delay(1000);
  chassis.waitUntilDone();
}

void leftSide(bool is_red_team)
{
  intake.setState(Intake::states::STORING);

  chassis.setPose({48 + 7.25 + 1., 24, -90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(24.75, 24.0, 2000, {}, false);
  chassis.turnToHeading(-180.0, 1000, {}, false);
  chassis.moveToPoint(24.75, -100.0, 1025, {.maxSpeed = 60, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  pros::delay(2200);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24., 48 - 7.25 + 2.5, left_goal.theta});

  // ? Grab Middle
  intake.setState(Intake::states::SCORING);
  chassis.moveToPose(36, 36, 90, 2000, {.lead = 0.9, .minSpeed = 20}, false);
  intake.setState(Intake::states::STORING);
  chassis.turnToHeading(46, 1000, {.minSpeed = 1}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(14 * sqrt(2), 46), 2200, {.maxSpeed = 50}, false);

  // ! Score Middle
  chassis.turnToHeading(48, 1000, {}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(19, 48), 2000, {.minSpeed = 45}, false);

  intake.setState(Intake::states::SCORE_MIDDLE);
  intake.setMiddle(true);
  pros::delay(1400);

  // * Retreat
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-45, 45), 2000, {.forwards = false, .minSpeed = 10}, false);

  // // ! Score Middle
  // chassis.turnToHeading(45, 1000, {}, false);
  // chassis.moveToPose(59.25, 62.75, 45, 1800, {.lead = 0.2}, false);
  // intake.setState(Intake::states::SCORE_MIDDLE);
  // intake.setMiddle(true);
  // pros::delay(1500);

  // // * Retreat and wack
  // intake.setMiddle(false);
  // chassis.turnToHeading(180, 2000, {.minSpeed = 1}, false);
}

void rightSide(bool is_red_team)
{
  intake.setState(Intake::states::STORING);

  chassis.setPose({-(48 + 7.25 + 1.), 24, 90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(-24.5, 24.0, 2000, {}, false);
  chassis.turnToHeading(179.0, 1000, {.minSpeed = 1}, false);
  chassis.moveToPoint(-24.5, -100.0, 1050, {.maxSpeed = 50, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x + 0.5, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  pros::delay(2500);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({-20., 48 - 7.25 + 3, left_goal.theta});

  // ? Grab Middle
  intake.setState(Intake::states::SCORING);
  chassis.moveToPose(-36, 36, -90, 2000, {.lead = 0.8, .minSpeed = 10}, false);
  intake.setState(Intake::states::STORING);
  chassis.turnToHeading(-44, 1000, {.minSpeed = 1}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(16 * sqrt(2), -44), 2200, {.maxSpeed = 30}, false);

  //! Score Middle
  chassis.turnToHeading(-43.5, 1000, {}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(19.0, -43.5), 1800, {.maxSpeed = 80}, false);
  intake.setState(Intake::states::OUTTAKE);

  chassis.moveToRelativePoint(Chassis::fromPolar(-4, -43.5), 1800, {.maxSpeed = 127}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(4, -43.5), 1800, {.maxSpeed = 80}, false);

  pros::delay(1400);

  // Retreat
  chassis.moveToRelativePoint(
      lemlib::Pose{0, 2, 0} + Chassis::fromPolar(38 * sqrt(2), 135),
      2000,
      {.forwards = false},
      false);

  chassis.turnToHeading(180, 2000, {.minSpeed = 1}, false);
}

void skills(bool is_red_team)
{
  intake.setState(Intake::states::STORING);
  chassis.setPose({49 + 5.75 + 0.5, 24, -90}, false);

  // UNLOAD GOAL 1
  matchload_unloader.extend();
  chassis.moveToPoint(24.5, 24, 2000, {}, false);

  // Grab Blocks
  chassis.turnToHeading(-180, 1000, {}, false);
  chassis.moveToPoint(24.5, -100.0, 2000, {.maxSpeed = 50, .minSpeed = 20}, false);

  // SCORE GOAL 1
  chassis.moveToRelativePoint(-0.25, 38, 1800, {.forwards = false, .maxSpeed = 100}, true);
  pros::delay(200);
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(2000);
  chassis.waitUntilDone();
  matchload_unloader.retract();

  // GO TO GOAL 2
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(0, -6, 1000, {.minSpeed = 10}, false);
  chassis.turnToHeading(-90, 1200, {.minSpeed = 1}, false);
  chassis.moveToRelativePoint(-12, 0, 1500, {}, false);
  chassis.turnToHeading(0, 1300, {}, true);

  // ! RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({14, 34}, 8.0);
  chassis.waitUntilDone();
  pros::delay(800);

  auto left_goal_1 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_1.theta);

  // Go to loader 2
  chassis.moveToPoint(12, 96, 2000, {.maxSpeed = 100, .minSpeed = 10}, false);
  chassis.turnToPoint(25.5, 120, 1500, {.minSpeed = 10}, false);
  chassis.moveToPoint(25.5, 120, 2000, {.maxSpeed = 80}, false);

  // Unload Loader 2
  chassis.turnToHeading(0, 1000, {}, false);
  matchload_unloader.extend();
  pros::delay(200);
  chassis.moveToPoint(25.5, 1000, 3500, {.maxSpeed = 60, .minSpeed = 20}, false);

  // SCORE GOAL 2
  chassis.moveToRelativePoint(0.0, -38, 1800, {.forwards = false, .maxSpeed = 100}, false);
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(2000);
  matchload_unloader.retract();

  // ! RESET TO LEFT GOAL 2
  auto left_goal_2 = chassis.getPose();
  chassis.setPose(24.5, 98.5, left_goal_2.theta);

  // GO TO GOAL 3
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(0, 8, 1000, {.minSpeed = 10}, false);

  // GO TO GOAL 3
  chassis.turnToPoint(116.5, 107, 1000, {.minSpeed = 10}, true);

  // Loader 3
  chassis.moveToPoint(116.5, 107, 2000, {.maxSpeed = 100, .minSpeed = 10}, false);

  chassis.turnToHeading(0, 1200, {}, false);
  matchload_unloader.extend();
  pros::delay(800);
  chassis.moveToPoint(116.5, 1000, 4000, {.maxSpeed = 60, .minSpeed = 20}, false);

  // GO to other side
  chassis.moveToRelativePoint(0, -12, 1500, {.forwards = false}, false);

  // ! RESET TO LEFT GOAL 2 using mcl (without back sensor)
  mcl_localization.reset_particles({120, 120}, 8.0);
  pros::delay(1000);

  auto left_goal_3 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_3.theta);

  // Go to side of goal
  chassis.turnToPoint(134, 96, 1500, {.forwards = false, .minSpeed = 10}, false);
  chassis.moveToPoint(134, 96, 1500, {.forwards = false, .minSpeed = 10}, false);

  // Drive down the alley
  chassis.turnToPoint(134, 48, 1500, {.forwards = false, .minSpeed = 10}, false);
  matchload_unloader.retract();
  chassis.moveToPoint(134, 48, 1500, {.forwards = false, .minSpeed = 10}, false);

  // Start aligning to score
  chassis.turnToPoint(120, 24, 1500, {.forwards = false}, false);
  chassis.moveToPoint(120, 24, 1500, {.forwards = false}, false);
  chassis.turnToHeading(180, 1000, {}, false);

  // SCORE GOAL 3
  chassis.moveToRelativePoint(0.0, 38, 1800, {.forwards = false, .maxSpeed = 100}, false);
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);
  intake.setState(Intake::states::STORING);

  // LOADER 4
  matchload_unloader.extend();
  chassis.moveToRelativePoint(0, -18.0, 1000, {.maxSpeed = 90, .minSpeed = 40}, false);
  chassis.moveToRelativePoint(0, -1000, 2500, {.maxSpeed = 50, .minSpeed = 20}, false);

  // Score Goal 4
  chassis.moveToRelativePoint(0.0, 38, 1800, {.forwards = false, .maxSpeed = 100}, false);
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);
  intake.setState(Intake::states::STORING);
}

void rightSideFinals(bool is_red_team)
{
  chassis.setPose({144 - 46, 24, 0});

  //* Pick Up 3 Cluster
  intake.setState(Intake::states::STORING);
  chassis.moveToPoint(144 - 46, 38, 2000, {.maxSpeed = 100, .minSpeed = 20}, false);
  chassis.swingToHeading(
      40, lemlib::DriveSide::RIGHT, 2000, {.maxSpeed = 60, .minSpeed = 20}, false);

  // ! Go to 2 Cluster
  chassis.moveToRelativePose(
      lemlib::Pose{5, 0, 90 - 40} + Chassis::fromPolar(31, 40, false),
      2000,
      {.lead = 0.6, .maxSpeed = 80},
      false);
  matchload_unloader.extend();
  pros::delay(300);

  // ? Go to Goal
  chassis.moveToRelativePose({0, -48, 180}, 2000, {.forwards = false, .minSpeed = 30}, false);
  chassis.moveToRelativePose(
      {16, 36, -95}, 3000, {.forwards = false, .lead = 0.9, .minSpeed = 10}, false);

  // * Score
  intake.setState(Intake::states::SCORING);
  matchload_unloader.retract();
  pros::delay(2000);

  // * Go to loader
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(0, -18, 2000, {}, false);
  matchload_unloader.extend();
  chassis.moveToRelativePoint(0, -100, 1200, {.maxSpeed = 40, .minSpeed = 20}, false);
}