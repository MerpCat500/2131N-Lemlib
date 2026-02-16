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
  chassis.moveToPose(42-0.5, 44 +0.5, 45, 2000, {.lead = 0.3, .minSpeed = 30}, false);
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

  chassis.moveToPoint(96, 52.5, 500, {.maxSpeed = 120, .minSpeed = 20}, false);
  chassis.moveToPoint(96, 52.5, 1000, {.maxSpeed = 40, .minSpeed = 20}, false);

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
  chassis.moveToPoint(24.75 +1, 24.0, 2000, {}, false);
  chassis.turnToHeading(-180.0, 1000, {}, false);
  chassis.moveToPoint(24.75 +1, -100.0, 1200, {.maxSpeed = 60, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x +1, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 45});
  pros::delay(600);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.23), 2000, {.forwards = false, .minSpeed = 25}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(750);
 intake.setState(Intake::states::OUTTAKE);
  pros::delay(300);
 intake.setState(Intake::states::SCORING);
  pros::delay(1800);
 
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24., 48 - 7.25 + 2.5, left_goal.theta});

  // ? Grab Middle
  intake.setState(Intake::states::SCORING);
  chassis.moveToPose(41, 33, 90, 2000, {.lead = 0.9, .minSpeed = 20}, false);
  intake.setState(Intake::states::STORING);
  chassis.turnToHeading(46, 1000, {.minSpeed = 1}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(14 * sqrt(2), 46), 2200, {.maxSpeed = 50}, false);

  // ! Score Middle
  chassis.turnToHeading(48 + 179, 1000, {}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(-12.5, 48+179), 2000, {.forwards = false, .minSpeed = 25}, false);

  
  middle_lift.retract();
  intake.setState(Intake::states::SCORING);
  intake.setMiddle(true);
  pros::delay(1400);

  // * Retreat
  // chassis.moveToRelativePoint(
  //     Chassis::fromPolar(-45, 45), 2000, {.forwards = false, .minSpeed = 10}, false);

//   //! swipe
//   chassis.moveToRelativePoint(
//       Chassis::fromPolar(-30.09, 45), 2100, {.forwards = false, .minSpeed = 10}, false);
//   chassis.turnToHeading(180, 1000);
//   pros::delay(400);
//   goal_descore_right.retract();
//   chassis.moveToRelativePoint(
//       Chassis::fromPolar(-20.5, 180), 2500, {.forwards = false, .maxSpeed = 44.5}, false);
//   goal_descore_right.extend();
  // // // ! Score Middle
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
   //chassis.moveToRelativePoint(Chassis::fromPolar(3, 0), 1200, {.maxSpeed = 100}, false);
   
  intake.setState(Intake::states::STORING);

 chassis.setPose({-(48 + 7.25 + 1.), 24, 90}, false);
 chassis.moveToRelativePoint(Chassis::fromPolar(3, 0), 1200, {.maxSpeed = 100}, false);
 /*  
  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(-25.35 -2.65, 24.0, 1000, {}, false);
  chassis.turnToHeading(179.0, 1000, {.minSpeed = 1}, false);
  chassis.moveToPoint(-25.25 -2.65, -100.0, 1100, {.maxSpeed = 50, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x + 0.5, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 60});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  pros::delay(2000);
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({-20., 48 - 7.25 + 3, left_goal.theta});

  // ? Grab Middle
  intake.setState(Intake::states::SCORING);
  chassis.moveToPose(-35, 35, -90, 2000, {.lead = 0.8, .minSpeed = 10}, false);
  intake.setState(Intake::states::STORING);
  chassis.turnToHeading(-44, 1000, {.minSpeed = 1}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(16 * sqrt(2), -44), 1800, {.maxSpeed = 30}, false);

  //! Score Middle
  first_stage_lift.extend();
  chassis.turnToHeading(-43.5, 500, {}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(19.5, -43.5), 1200, {.maxSpeed = 80}, false);
  intake.setState(Intake::states::OUTTAKE);

  chassis.moveToRelativePoint(Chassis::fromPolar(-4.5, -43.5), 1200, {.maxSpeed = 127}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(4, -43.5), 1200, {.maxSpeed = 80}, false);

  pros::delay(300);

  // Retreat
  // chassis.moveToRelativePoint(
  //     lemlib::Pose{0, 2, 0} + Chassis::fromPolar(38 * sqrt(2), 135),
  //     2000,
  //     {.forwards = false},n  
  //     false);
  first_stage_lift.retract();
  //! swipe
  //chassis.moveToPoint(-35.5, 45.5, 1000, {.forwards = false});
  //chassis.turnToHeading(179, 800);  
  //goal_descore_left.retract();
  //chassis.moveToRelativePoint(0.1, -10, 2000, {.forwards = false}, false);
  //chassis.turnToPoint(-27, 55, 1500, {.minSpeed = 30}, false);
  chassis.moveToPoint(-27, 55, 1000, {.forwards = false, .minSpeed = 45});

  //pros::delay(400);
  //chassis.turnToHeading(188, 1000);
  //goal_descore_left.extend();
  */
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
  chassis.moveToPoint(24.5-0.1, -100.0, 2500, {.maxSpeed = 50, .minSpeed = 20}, false);

  chassis.moveToRelativePoint(0, 20, 2000, {.forwards = false}, false);
  intake.setState(Intake::states::STOPPED);
  matchload_unloader.retract();
  chassis.turnToHeading(-90, 1800, {}, false);
  chassis.moveToRelativePoint(-10, 0, 2000, {}, false);

  // ! RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({12, 34}, 8.0);
  pros::delay(500);

  auto left_goal_1 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_1.theta);

  // Go to loader 2
  chassis.moveToPoint(11, 96, 2000, {.minSpeed = 20}, false);
  chassis.turnToPoint(23.90, 120, 1500, {.minSpeed = 30}, false);
  chassis.moveToPoint(23.90, 120, 2000, {}, false);

  // Score Loader 1
  chassis.turnToHeading(0, 1000, {}, false);
  intake.setState(Intake::states::STORING);  // STops bottom block from yeeting out
  chassis.moveToRelativePoint(-1.8, -38, 2000, {.forwards = false}, true);
  pros::delay(400);
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(1500);

  // ! PIck up strays
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(0, 6, 800, {.minSpeed = 30}, false);
  chassis.moveToRelativePoint(0, -6, 1000, {.forwards = false, .minSpeed = 30}, false);

  // Score again
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(2000);
  intake.setState(Intake::states::STORING);

  matchload_unloader.extend();
  pros::delay(200);
  chassis.moveToPoint(27.25-.25, 1000, 2500, {.maxSpeed = 60, .minSpeed = 20}, false);
  chassis.turnToHeading(-10, 700, {.minSpeed = 20});
  chassis.turnToHeading(10, 700, {.minSpeed = 20});
  chassis.turnToHeading(0, 700, {.minSpeed = 20});

  // SCORE GOAL 2
  chassis.moveToRelativePoint(-1, -38, 1800, {.forwards = false, .maxSpeed = 100}, false);
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(2500);
  chassis.moveToRelativePoint(0, 6, 800, {.minSpeed = 30}, false);
  chassis.moveToRelativePoint(-.25, -6, 1000, {.forwards = false, .minSpeed = 30}, false);
  intake.antiJam(true);
  pros::delay(1800);
  matchload_unloader.retract();

  // ! RESET TO LEFT GOAL 2
  auto left_goal_2 = chassis.getPose();
  chassis.setPose(24.5, 98.5, left_goal_2.theta);

  // GO TO GOAL 3
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(-1.5, 8, 1000, {.minSpeed = 10}, false);

  // GO TO GOAL 3
  chassis.turnToPoint(112+3, 107, 1000, {.minSpeed = 10}, true);

  // Loader 3
  chassis.moveToPoint(112+3, 107, 2000, {.minSpeed = 10}, false);

  chassis.turnToHeading(0, 1200, {}, false);
  matchload_unloader.extend();
  pros::delay(800);
  chassis.moveToPoint(114.3, 1000, 2000, {.maxSpeed = 60, .minSpeed = 23}, false);
  chassis.turnToHeading(10, 500, {.minSpeed = 20});
  chassis.turnToHeading(-10, 600, {.minSpeed = 20});
  chassis.turnToHeading(0, 800, {.minSpeed = 21});

  // GO to other side
  chassis.moveToRelativePoint(0.3, -12, 1500, {.forwards = false}, false);
  matchload_unloader.retract();
  chassis.turnToHeading(90, 1800, {.minSpeed = 10}, false);
  chassis.moveToRelativePoint(12+1.5, 0, 2000, {}, false);

  // ! RESET TO LEFT GOAL 2 using mcl (without back sensor)
  mcl_localization.reset_particles({132, 120}, 8.0);
  pros::delay(750);

  auto left_goal_3 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_3.theta);

  intake.setState(Intake::states::STOPPED);

  // Drive down the alley
  chassis.turnToPoint(141, 48, 1500, {.minSpeed = 20}, false);
  chassis.moveToPoint(141, 48, 1500, {.minSpeed = 10}, false);

  // Start aligning to score
  chassis.turnToPoint(124.5, 24, 1500, {.minSpeed = 50}, false);
  chassis.moveToPoint(124.5, 24, 1500, {}, false);
  chassis.turnToHeading(180, 1000, {}, false);

  // SCORE GOAL 3
  mcl_localization.reset_particles({123, 24}, 8.0);
  pros::delay(500);

  auto right_goal_1 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_goal_1.theta);

  chassis.moveToRelativePoint(1, 38, 1800, {.forwards = false, .maxSpeed = 100}, false);
  //chassis.moveToPoint(124 -0.5, 24 +38, 1500, {.forwards = false}, false);
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(2000);
  intake.antiJam(true);
  pros::delay(1500);
  intake.setState(Intake::states::STORING);

  // LOADER 4
  matchload_unloader.extend();
  chassis.moveToRelativePoint(-1.6, -18.0, 800, {.maxSpeed = 90, .minSpeed = 40}, false);
  chassis.moveToRelativePoint(-1.6, -1000, 2500, {.maxSpeed = 50, .minSpeed = 20}, false);


  // Score Goal 4
  chassis.moveToRelativePoint(3.7, 38, 1800, {.forwards = false, .maxSpeed = 100}, false);
  //hassis.moveToPoint(124 -0.5, 24 +38, 1500, {.forwards = false}, false);
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(900);
  intake.antiJam(true);
  pros::delay(1200);
  intake.setState(Intake::states::STORING);

  // Park
  matchload_unloader.retract();
  auto right_goal_2 = chassis.getPose(false);
  chassis.setPose(120, 40, right_goal_2.theta);
  chassis.moveToPoint(108, 14-0.5, 2000, {}, false);
  chassis.turnToHeading(-100, 1000);
  chassis.moveToRelativePoint(Chassis::fromPolar(50, -100), 2000, {.minSpeed = 80}, false);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-5, -100), 2000, {.forwards = false, .minSpeed = 60}, false);
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

void Safeskills(bool is_red_team)
{
  intake.setState(Intake::states::STORING);

  chassis.setPose({-(48 + 7.25 + 1.), 24, 90}, false);

  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(-24.5, 24.0, 2000, {.maxSpeed = 65}, false);
  chassis.turnToHeading(179.0, 1000, {.minSpeed = 1}, false);
  chassis.moveToPoint(-24.5, -100.0, 3000, {.maxSpeed = 50, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(
      after_loader.x + 0.5, after_loader.y + 36.0, 1800, {.forwards = false, .maxSpeed = 80});
  pros::delay(600);
  intake.setState(Intake::states::SCORING);
  pros::delay(1000);
  intake.setState(Intake::states::OUTTAKE);
  pros::delay(200);
  intake.setState(Intake::states::SCORING);
  pros::delay(750);
  matchload_unloader.retract();
  chassis.moveToPoint(
      after_loader.x + 0.5, after_loader.y + 30.0, 1000, {.forwards = false, .maxSpeed = 80});
  pros::delay(500);
  chassis.moveToPoint(
      after_loader.x + 0.5, after_loader.y + 36.0, 1000, {.forwards = false, .maxSpeed = 80});
  intake.setState(Intake::states::SCORING);
  pros::delay(500);
  //! moving away from goal
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(Chassis::fromPolar(-16, 0), 2000, {}, false);
  chassis.turnToHeading(270, 500);
  chassis.moveToRelativePoint(Chassis::fromPolar(9.5, 270), 2000, {}, false);
  chassis.turnToHeading(328, 500);
  chassis.moveToRelativePoint(Chassis::fromPolar(31, 328), 2000, {.maxSpeed = 38}, false);
  pros::delay(300);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-31, 328), 2000, {.forwards = false, .maxSpeed = 50}, false);
  chassis.turnToHeading(270, 400);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-9.5, 270), 2000, {.forwards = false, .maxSpeed = 59}, false);
  chassis.turnToHeading(180, 500);
  chassis.moveToRelativePoint(Chassis::fromPolar(-17, 180), 2000, {.forwards = false}, false);
  //! scoring second time first goal

  intake.setState(Intake::states::SCORING);
  pros::delay(1400);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(13, 180), 2000, {.forwards = true, .maxSpeed = 60}, false);
  chassis.turnToHeading(270, 600);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(95, 270), 4000, {.forwards = true, .maxSpeed = 70}, false);
  chassis.turnToHeading(180, 500);
  matchload_unloader.extend();
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(167.5, 180), 2000, {.forwards = true, .minSpeed = 30}, false);
  pros::delay(1500);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-34, 180), 2000, {.forwards = false, .maxSpeed = 60}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(1000);
  intake.setState(Intake::states::OUTTAKE);
  pros::delay(200);
  intake.setState(Intake::states::SCORING);
  pros::delay(1500);
  matchload_unloader.retract();
  intake.setState(Intake::states::STORING);

  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(Chassis::fromPolar(-16, 0), 2000, {}, false);
  chassis.turnToHeading(-270, 500);
  chassis.moveToRelativePoint(Chassis::fromPolar(9.5, -270), 2000, {}, false);
  chassis.turnToHeading(-328, 500);
  chassis.moveToRelativePoint(Chassis::fromPolar(31, -328), 2000, {.maxSpeed = 38}, false);
  pros::delay(300);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-31, -328), 2000, {.forwards = false, .maxSpeed = 50}, false);
  chassis.turnToHeading(-270, 400);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-9.5, -270), 2000, {.forwards = false, .maxSpeed = 59}, false);
  chassis.turnToHeading(-180, 500);
  chassis.moveToRelativePoint(Chassis::fromPolar(-17, -180), 2000, {.forwards = false}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(2000);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(17, -180), 2000, {.forwards = true, .maxSpeed = 59}, false);
  chassis.turnToHeading(-235, 400);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(16, -235), 2000, {.forwards = true, .maxSpeed = 59}, false);
  chassis.turnToHeading(-255 + 180, 500);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-40, -255 + 180), 2000, {.forwards = false, .minSpeed = 72}, false);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-20, -255 + 180), 2000, {.forwards = false, .minSpeed = 70}, false);
  pros::delay(100);
  chassis.moveToRelativePoint(
      Chassis::fromPolar(-20, -255 + 180), 2000, {.forwards = false, .minSpeed = 70}, false);
  intake.setState(Intake::states::SCORE_MIDDLE);
  // chassis.moveToPoint(-11, 20, 1000, {.forwards = false});
  //    //! moving away from goal and swiping first one
  //    chassis.moveToPoint(
  //        after_loader.x + 0.5, after_loader.y + 26.0, 1000, {.forwards = false, .maxSpeed = 80});
  //    chassis.moveToPoint(
  //        after_loader.x - 9, after_loader.y + 28.0, 1000, {.forwards = false, .maxSpeed = 80});
  //    chassis.turnToHeading(180, 1000);
  //    goal_descore_left.retract();
  //    chassis.moveToPose(
  //        after_loader.x - 9, after_loader.y + 39.0, 180, 1000, {.forwards = false, .maxSpeed =
  //        60});
  //    chassis.moveToPose(
  //        after_loader.x - 9, after_loader.y + 47.0, 180, 2000, {.forwards = false, .maxSpeed =
  //        50});
}