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
  // * === Set the Inital State of the Robot === * //
  // Intake should store blocks
  intake.setState(Intake::states::STORING);

  // Starting To the left of the park facing the left wall
  chassis.setPose({48 + 7.25 + 1.5, 24, -90}, false);

  // ? === De-score the First Loader === ? //
  // Extend the matchload unloader
  matchload_unloader.extend();

  // Move and turn to the loader
  chassis.moveToPoint(26, 24.0, 2000, {}, false);
  chassis.turnToHeading(-180.0, 1000, {.minSpeed = 1}, false);

  // Drive into loader to grab blocks
  chassis.moveToPoint(
      26,
      -100.0,
      1100,
      {.maxSpeed = 60,
       .minSpeed = 30},  // Clamp max speed to stop robot from launching blocks out of the field
      false);

  // ! === Attempt to Score in Left Goal === ! //
  // Grab position after loader
  auto after_loader = chassis.getPose();

  // Move backwards ~ 2 Feild tiles (48 inches) to align to goal
  chassis.moveToPoint(
      after_loader.x, after_loader.y + 48.0, 1800, {.forwards = false, .maxSpeed = 68}, false);

  // Retract the matchload unloader
  matchload_unloader.retract();

  // * === Reckon to Left Goal === * //
  // Grab position after scoring
  auto left_goal = chassis.getPose();

  // Reset position to the goal, keep theta measurement the same.
  chassis.setPose({24., 48 - 7.25 + 3.5, left_goal.theta});

  // ? === Grab Top Middle Blocks === ? //
  // Un-jam anything in the intake
  intake.antiJam(true);

  // Move forward out of the goal as to not get stuck
  chassis.moveToRelativePoint(0., -8, 2000, {.minSpeed = 40}, false);

  // Set intake state to store blocks
  intake.setState(Intake::states::STORING);
  intake.setIntakeMultiplier(1.0);

  // Start swing motion at full speed, exit and slow down to .maxSpeed = 30
  chassis.moveToPose(48, 48, 10, 800, {.lead = 0.6}, false);
  chassis.moveToPose(48, 48, 10, 1000, {.lead = 0.6, .maxSpeed = 30}, false);

  // ! === Score Top in Middle === ! //
  // Turn to face the top middle goal
  chassis.turnToHeading(45, 1000, {.maxSpeed = 80}, false);

  // Set intake state (starts cycling blocks up)
  intake.setState(Intake::states::SCORE_MIDDLE);

  // Move to the middle goal
  chassis.moveToPose(60.5, 61, 45, 1800, {.lead = 0.2, .minSpeed = 10}, false);

  // Wait a little bit, open the middle goal flap to score
  pros::delay(200);
  intake.setMiddle(true);

  // Try to score for about a second, then stop intake and leave
  pros::delay(1000);
  intake.setState(Intake::states::STOPPED);

  // ? === Grab Bottom Middle Blocks === ? //
  // Look towards the Right Alliance Side Corner
  chassis.swingToHeading(135, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 30}, false);

  // Close Middle Goal Flap and Start Storing
  intake.setMiddle(false);
  intake.setState(Intake::states::STORING);

  // Start moving towards the bottom middle blocks, then slow down to pick them up.
  chassis.moveToRelativePose({48, -10, -45}, 700, {.lead = 0.4}, false);
  chassis.moveToRelativePose(
      {48, 10, 90}, 2000, {.lead = 0.4, .maxSpeed = 40, .minSpeed = 1}, false);

  // ! === Score Middle Bottom === ! //
  // Turn to face the middle goal
  chassis.turnToHeading(-43, 1500, {.minSpeed = 1}, false);

  // Move in a straight line to the middle goal
  chassis.moveToRelativePoint(Chassis::fromPolar(16.5, -43), 2000, {}, true);

  // Start out-taking slowly to not launch blocks through the goal
  intake.setIntakeMultiplier(0.8);
  intake.setState(Intake::states::OUTTAKE);

  // When the chassis has stopped moving, set intake to full speed.
  chassis.waitUntilDone();
  intake.setIntakeMultiplier(1.0);
}

void leftSide(bool is_red_team)
{
  // * === Set the Initial State of the Robot === * //
  // Intake should store blocks
  intake.setState(Intake::states::STORING);

  // Starting to the left of the park facing the left wall
  chassis.setPose({48 + 7.25 + 1., 24, -90}, false);

  // ? === De-score loader === ? //
  // Extend the matchload unloader
  matchload_unloader.extend();

  // Drive and turn to the loader
  chassis.moveToPoint(24.75, 24.0, 2000, {}, false);
  chassis.turnToHeading(-180.0, 1000, {}, false);

  // Drive into the loader to grab blocks, use max speed to stop excessive force
  chassis.moveToPoint(24.75, -100.0, 1025, {.maxSpeed = 60, .minSpeed = 20}, false);

  // ! === Attempt to score === ! //
  // Grab position after loader
  auto after_loader = chassis.getPose();

  // Move backwards 36 Inches relative to the loader
  chassis.moveToPoint(
      after_loader.x, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 80});

  // Delay to get to the goal
  pros::delay(600);

  // Start Scoring for 2.2 seconds
  intake.setState(Intake::states::SCORING);
  pros::delay(2200);

  // Reset the matchloader
  matchload_unloader.retract();

  // * === Reset to the goal === * //
  auto left_goal = chassis.getPose();
  chassis.setPose({24., 48 - 7.25 + 2.5, left_goal.theta});

  // ? Grab Middle Blocks
  // Anti-jam any stuck blocks
  intake.antiJam(true);

  // Move out of the goal
  chassis.moveToPose(36, 36, 90, 2000, {.lead = 0.9, .minSpeed = 10}, false);

  // Set intake to storing
  intake.setState(Intake::states::STORING);

  // Look towards middle goal and move towards it slowly
  chassis.turnToHeading(46, 1000, {.minSpeed = 1}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(14 * sqrt(2), 46), 2200, {.maxSpeed = 30}, false);

  // ! Score Middle Blocks
  // Ensure the robot is still looking at the middle goal
  chassis.turnToHeading(45, 1000, {}, false);

  // Move to the middle goal and score in the middle goal for 1.4 seconds
  chassis.moveToRelativePoint(Chassis::fromPolar(21.5, 45), 1800, {.maxSpeed = 60}, false);
  intake.setState(Intake::states::SCORE_MIDDLE);
  intake.setMiddle(true);
  pros::delay(1400);

  // * === Retreat to Corner to Start Driver === * //
  // Back up 45 inches in the direction the robot is facing.
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-45, 45), 2000, {.forwards = false, .minSpeed = 10}, false);

  // Reset intake and face the loader to prepare for driver control
  intake.setMiddle(false);
  chassis.turnToHeading(180, 2000, {.minSpeed = 1}, false);
}

void rightSide(bool is_red_team)
{
  // * === Set the Initial State of the Robot === * //
  // Intake should store blocks
  intake.setState(Intake::states::STORING);

  // Starting to the right of the park facing the right wall
  chassis.setPose({-(48 + 7.25 + 1.), 24, 90}, false);

  // ? === De-score loader === ? //
  // Extend the matchload unloader
  matchload_unloader.extend();

  // Move and turn to the loader
  chassis.moveToPoint(-24.5, 24.0, 2000, {}, false);
  chassis.turnToHeading(179.0, 1000, {.minSpeed = 1}, false);

  // Drive into the loader to grab blocks, implement max speed to stop excessive force launching
  // blocks
  chassis.moveToPoint(-24.5, -100.0, 1050, {.maxSpeed = 50, .minSpeed = 20}, false);

  // ! === Attempt to score === ! //
  // Grab position after loader
  auto after_loader = chassis.getPose();

  // Move backwards 36 Inches relative to the loader
  chassis.moveToPoint(
      after_loader.x + 0.5, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 80});

  // Delay to get to the goal
  pros::delay(600);

  // Start Scoring for 2.5 seconds
  intake.setState(Intake::states::SCORING);
  pros::delay(2500);

  // Reset the matchloader
  matchload_unloader.retract();

  // * === Reset to the goal === * //
  // Grab left
  auto right_goal = chassis.getPose();
  chassis.setPose({-20., 48 - 7.25 + 3, right_goal.theta});

  // ? === Grab Middle Blocks === ? //
  // Un-jam any stuck blocks
  intake.antiJam(true);

  // Move out of the goal
  chassis.moveToPose(-36, 36, -90, 2000, {.lead = 0.8, .minSpeed = 10}, false);

  // Set intake to storing
  intake.setState(Intake::states::STORING);

  // Look towards middle goal and move towards it slowly
  chassis.turnToHeading(-44, 1000, {.minSpeed = 1}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(16 * sqrt(2), -44), 2200, {.maxSpeed = 30}, false);

  //! === Score Middle Blocks === ! //
  // Ensure the robot is still looking at the middle goal
  chassis.turnToHeading(-43.5, 1000, {}, false);

  // Move to the middle goal and score in the middle goal
  chassis.moveToRelativePoint(Chassis::fromPolar(19.0, -43.5), 1800, {.maxSpeed = 80}, false);

  // Wait for 1.4 seconds while out-taking
  intake.setState(Intake::states::OUTTAKE);
  pros::delay(1400);

  // Ram the middle bottom goal to try and get any last blocks in
  chassis.moveToRelativePoint(Chassis::fromPolar(-4, -43.5), 1800, {.maxSpeed = 127}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(4, -43.5), 1800, {.maxSpeed = 80}, false);

  // Retreat to the corner to start driver control
  chassis.moveToRelativePoint(
      lemlib::Pose{0, 2, 0} + Chassis::fromPolar(38 * sqrt(2), 135),
      2000,
      {.forwards = false},
      false);

  // Look at loader if time remains
  chassis.turnToHeading(180, 2000, {.minSpeed = 1}, false);
}

void skills(bool is_red_team)
{
  intake.setState(Intake::states::STORE_TOP);

  chassis.setPose({144 - (48 + 7.25 + 1.), 24, 90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(144 - 23.5, 24.0, 2000, {}, false);
  chassis.turnToHeading(180, 1000, {}, false);
  chassis.moveToPoint(144 - 23.5, -100.0, 1400, {.maxSpeed = 40, .minSpeed = 10}, false);
  chassis.moveToPoint(144 - 23.5, 7.0, 1000, {.maxSpeed = 40, .minSpeed = 10}, false);
  chassis.moveToPose(144 - 23.5, -10, 175, 400);
  chassis.moveToPoint(144 - 23, -10.0, 600, {.maxSpeed = 40, .minSpeed = 10}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x - 0.75, after_loader.y + 36.0, 5000, {.forwards = false, .maxSpeed = 70});
  pros::delay(675);
  intake.antiJam(true);
  intake.setState(Intake::states::SCORING);
  pros::delay(5000);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto right_goal = chassis.getPose();
  chassis.setPose({144 - 24 + 1.5, 48 + 7.5, right_goal.theta});

  // ! Go to loader 2
  intake.setState(Intake::states::OUTTAKE);
  chassis.moveToPose(96, 36, -90, 2000, {.lead = (0.70710678118), .minSpeed = 40}, false);
  intake.setState(Intake::states::STORE_TOP);
  matchload_unloader.extend();
  chassis.moveToPose(27, 12, 180, 5000, {.lead = 0.65}, false);

  // ! Attempt to score
  auto after_loader2 = chassis.getPose();
  chassis.moveToPoint(
      after_loader2.x, after_loader2.y + 36.0, 5000, {.forwards = false, .maxSpeed = 70});
  pros::delay(675);
  intake.antiJam(true);
  intake.setIntakeMultiplier(1.0);
  intake.setState(Intake::states::SCORING);
  pros::delay(1000);
  intake.setIntakeMultiplier(0.8);
  pros::delay(4000);
  matchload_unloader.retract();
  pros::delay(800);

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24, 48 + 7.5, left_goal.theta});

  // ! PARK
  chassis.moveToRelativePoint(0, -5, 2000, {.minSpeed = 30}, false);
  chassis.moveToPoint(36, 18, 2000, {}, false);
  chassis.turnToHeading(100, 1000);
  chassis.moveToRelativePoint(Chassis::fromPolar(58, 100), 2000, {.minSpeed = 60}, true);
  pros::delay(800);
  intake.setState(Intake::states::OUTTAKE);
  intake.setMiddle(true);

  //   chassis.moveToPose(48, 20, 90, 2000, {.lead = 0.7, .minSpeed = 80}, false);
  //   intake.setState(Intake::states::OUTTAKE);
  //   intake.setMiddle(true);
  //   chassis.moveToPose(104, 11, 120, 2000, {.minSpeed = 80});
  //   pros::delay(400);
  //   chassis.moveToPoint(104, 11, 2000, {});
  //   matchload_unloader.extend();
  //   chassis.waitUntilDone();
  //   matchload_unloader.retract();
}

void skills2(bool is_red_team)
{
  intake.setState(Intake::states::STORING);

  chassis.setPose({144 - (48 + 7.25 + 1.5), 24, 90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(144 - 24, 24.0, 2000, {}, false);
  chassis.turnToHeading(180, 1000, {}, false);
  chassis.moveToPoint(144 - 24, -100.0, 3000, {.maxSpeed = 40, .minSpeed = 30}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x - 0.75, after_loader.y + 36.0, 5000, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.antiJam(true);
  pros::delay(100);
  intake.setState(Intake::states::SCORING);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto right_goal = chassis.getPose();
  chassis.setPose({144 - 24, 48 - 7.5, right_goal.theta});

  // ? Leave Goal
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(0, -4, 1000, {.minSpeed = 40}, false);
  chassis.swingToHeading(
      0,
      lemlib::DriveSide::LEFT,
      1500,
      {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 10},
      false);

  intake.setState(Intake::states::OUTTAKE);
  chassis.moveToPoint(144 - 12, 96, 2000, {.minSpeed = 30}, false);
  intake.setState(Intake::states::STORING);

  // ! Unload Loader 2
  matchload_unloader.extend();
  chassis.moveToPose(119, 125, 0, 2000, {.lead = 0.4, .minSpeed = 10}, false);
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(0, 100, 3000, {.maxSpeed = 40, .minSpeed = 30}, false);

  // * Score Loader 2
  chassis.moveToRelativePoint(0, -48, 2000, {.forwards = false, .maxSpeed = 70}, true);
  pros::delay(600);
  intake.antiJam(true);
  pros::delay(100);
  intake.setState(Intake::states::SCORING);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto right_goal2 = chassis.getPose();
  chassis.setPose({144 - 24, 96 + 7.5, right_goal2.theta});

  // ? Go to Loader 3
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(0, 4, 1000, {.minSpeed = 40}, false);
  chassis.moveToPoint(72, 108, 3000, {.minSpeed = 40}, false);
  chassis.moveToPose(23, 124, 0, 3000, {.lead = 0.2}, false);

  // ! Unscore loader 3
  matchload_unloader.extend();
  chassis.moveToRelativePoint(0, 100, 3000, {.maxSpeed = 40, .minSpeed = 30}, false);

  // * Score Loader 3
  chassis.moveToRelativePoint(0, -48, 2000, {.forwards = false, .maxSpeed = 70}, true);
  pros::delay(600);
  intake.antiJam(true);
  pros::delay(100);
  intake.setState(Intake::states::SCORING);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({26, 96 + 7.5, left_goal.theta});
  chassis.moveToPoint(26, 100 + 7.5, 1000, {.minSpeed = 40}, false);

  // ? Leave Goal 3
  matchload_unloader.retract();
  intake.setState(Intake::states::STORING);
  //   chassis.moveToRelativePoint(0, 4, 1000, {.minSpeed = 40}, false);
  chassis.swingToHeading(
      180,
      lemlib::DriveSide::LEFT,
      1500,
      {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 10},
      false);
  chassis.moveToPoint(12, 48, 2000, {.minSpeed = 30}, false);

  // ! Unload Loader 4
  matchload_unloader.extend();
  chassis.moveToPose(26, 18, 180, 3000, {.lead = 0.4, .minSpeed = 10}, false);
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(0, -100, 3000, {.maxSpeed = 40, .minSpeed = 30}, false);

  // ? Score Loader 4
  chassis.moveToRelativePoint(2, 48, 2000, {.forwards = false, .maxSpeed = 70}, true);
  pros::delay(600);
  intake.antiJam(true);
  pros::delay(100);
  intake.setState(Intake::states::SCORING);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);
  intake.antiJam(true);
  pros::delay(1000);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto left_goal2 = chassis.getPose();
  chassis.setPose({24, 48 + 7.5, left_goal2.theta});

  // ! PARK
  chassis.moveToPose(12, 27, -90, 2000, {.lead = 0.8}, false);
  intake.setState(Intake::states::SCORE_MIDDLE);
  pros::delay(200);
  chassis.turnToPoint(80, 0, 800, {}, false);
  chassis.moveToPoint(92, 10, 8000, {.maxSpeed = 127, .minSpeed = 127});

  matchload_unloader.extend();
  intake.setMiddle(true);

  chassis.waitUntilDone();
  matchload_unloader.retract();
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