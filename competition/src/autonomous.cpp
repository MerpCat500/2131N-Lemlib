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
middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({87.75, 24, 90}, false);
  
  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(119.75, 24.0, 1500, {.maxSpeed = 60, .minSpeed = 40}, false);
  chassis.turnToHeading(180.0, 500, {.maxSpeed = 60, .minSpeed = 35}, false);
   chassis.moveToPoint(119.75, -100.0, 930, {.maxSpeed = 60, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(after_loader.x -1.2, after_loader.y + 36.0, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 60});
  pros::delay(0);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.23), 100, {.forwards = false, .minSpeed = 50}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(300);
 //intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
 intake.setState(Intake::states::SCORING);
  pros::delay(900);
 
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({119.75, 48 - 7.25 + 2.5, left_goal.theta});

   // ? Grab Middle
  intake.setState(Intake::states::SCORING);
  chassis.moveToPose(102, 36.5, -90, 1000, {.lead = 0.9, .minSpeed = 40}, false);
  intake.setState(Intake::states::STORING);
  chassis.turnToPoint(82, 46.25, 500, {.minSpeed = 30});
  chassis.moveToPoint(82, 46.25, 1000, {.maxSpeed = 50, .minSpeed = 35}, false);

  chassis.turnToPoint(45, 46.25, 500, {.minSpeed = 30});
  chassis.moveToPoint(45, 46.25, 1300, {.maxSpeed = 60, .minSpeed = 40}, false);
  chassis.moveToPoint(49, 46.25, 160, {.forwards = false, .maxSpeed = 70, .minSpeed = 50}, false);
// line up to score
  chassis.turnToPoint(58+4, 58-1, 300, {.forwards = false, .minSpeed = 40});
  chassis.moveToPoint(58+4, 58-1, 700, {.forwards = false,  .minSpeed = 55}, false);


  // ! Score Middle
  middle_lift.retract();
  intake.setIntakeMultiplier(1.0, 1.0, 0.29);
  intake.setState(Intake::states::SCORING);
  intake.setMiddle(true);
  pros::delay(460);
  intake.setIntakeMultiplier(1.0, 1.0, 1.0);
  intake.setState(Intake::states::STORING);

  // ? go to other loader 

  matchload_unloader.extend();

  chassis.turnToPoint(24+7, 24, 500, { .minSpeed = 40});
  chassis.moveToPoint(24+7, 24, 1300, { .maxSpeed = 90, .minSpeed = 70}, false);

  // * loader clear 


  chassis.turnToHeading(180.0, 500, {.minSpeed = 40}, false);
  

   chassis.moveToPoint(24+7, -100.0, 550, {.maxSpeed = 60, .minSpeed = 20}, false);

  // ! Attempt to score
//   auto after_loader2 = chassis.getPose();
//   chassis.moveToPoint(after_loader2.x -1.2, after_loader2.y + 36.0, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 90});
  chassis.moveToRelativePoint(0, -40, 1000, {.forwards = false, .minSpeed = 80}, false);
  middle_lift.extend();
  pros::delay(800);
  intake.setState(Intake::states::SCORING);
  pros::delay(300);
 //intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
 intake.setState(Intake::states::SCORING);
  pros::delay(900);
  matchload_unloader.retract();
/////









//   intake.setState(Intake::states::STORE_TOP);
//   chassis.setPose({49 + 5.75 + 1, 24, -90}, false);

//   // UNLOAD GOAL 1
//   matchload_unloader.extend();
//   chassis.moveToPoint(24, 24, 2000, {}, false);

//   // Grab Blocks
//   chassis.turnToHeading(-180, 1000, {.minSpeed = 1}, false);
//   chassis.moveToPoint(24, -100.0, 1200, {.maxSpeed = 80, .minSpeed = 30}, false);

//   // SCORE GOAL 1
//   chassis.moveToRelativePoint(0.0, 38, 1500, {.forwards = false, .maxSpeed = 100}, true);
//   pros::delay(500);
//   intake.setState(Intake::states::SCORING);
//   pros::delay(1000);
//   chassis.waitUntilDone();
//   matchload_unloader.retract();

//   // ! RESET TO LEFT GOAL 1
//   auto left_goal_1 = chassis.getPose();
//   chassis.setPose(23.5, 40.5, left_goal_1.theta);

//   // GO TO GOAL 2
//   chassis.moveToRelativePoint(0, -6, 1000, {.minSpeed = 10}, true);

//   chassis.turnToHeading(90, 1000, {.minSpeed = 1}, false);

//   // Go to left middle cluster
//   chassis.moveToPose(42-0.5, 44 +0.5, 45, 2000, {.lead = 0.3, .minSpeed = 30}, false);
//   intake.setState(Intake::states::STORING);

//   // Pick up blocks
//   chassis.moveToRelativePoint(
//       Chassis::fromPolar(6 * sqrt(2), 45), 3000, {.maxSpeed = 50, .minSpeed = 50}, false);
//   chassis.moveToRelativePoint(Chassis::fromPolar(16 * sqrt(2), 45), 1000, {}, true);
//   pros::delay(800);
//   intake.setState(Intake::states::SCORE_MIDDLE);

//   // Score in goal
//   chassis.waitUntilDone();
//   intake.setMiddle(true);
//   pros::delay(1000);
//   intake.setState(Intake::states::STORING);

//   // Retreat to next cluster
//   chassis.moveToPoint(48, 52, 2000, {.forwards = false, .minSpeed = 10}, false);
//   intake.setMiddle(false);

//   // Go to right middle cluster
//   chassis.turnToPoint(96, 52, 1500, {.minSpeed = 10}, false);

//   chassis.moveToPoint(96, 52.5, 500, {.maxSpeed = 120, .minSpeed = 20}, false);
//   chassis.moveToPoint(96, 52.5, 1000, {.maxSpeed = 40, .minSpeed = 20}, false);

//   // Final Loader
//   chassis.turnToPoint(117, 36, 1500, {.minSpeed = 10}, false);
//   chassis.moveToPoint(117, 36, 2000, {.maxSpeed = 100, .minSpeed = 20}, false);

//   chassis.turnToHeading(180, 1200, {}, false);
//   chassis.moveToRelativePoint(0.0, 38, 1800, {.forwards = false, .maxSpeed = 100}, true);
//   pros::delay(300);
//   intake.setState(Intake::states::SCORING);
//   pros::delay(1000);
//   chassis.waitUntilDone();
}

void leftSide(bool is_red_team)
{
  middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({48 + 7.25 + 1., 24, -90}, false);
  
  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(24.75 -.5, 24.0, 2000, {}, false);
  chassis.turnToHeading(-180.0, 1000, {}, false);
  chassis.moveToPoint(24.75-.5, -100.0, 1000, {.maxSpeed = 60, .minSpeed = 20}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(after_loader.x -1.2, after_loader.y + 36.0, 1600, {.forwards = false, .maxSpeed = 75, .minSpeed = 30});
  pros::delay(600);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.23), 600, {.forwards = false, .minSpeed = 50}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(300);
 intake.setState(Intake::states::OUTTAKE);
  pros::delay(300);
 intake.setState(Intake::states::SCORING);
  pros::delay(1300);
 
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24+.5,48 - 7.25 + 2.5, left_goal.theta});

  // ? Grab Middle
  intake.setState(Intake::states::SCORING);
  chassis.moveToPose(42, 36.5, 90, 2000, {.lead = 0.9, .minSpeed = 20}, false);
  intake.setState(Intake::states::STORING);
  chassis.turnToHeading(46, 1000, {.minSpeed = 10}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(14 * sqrt(2), 46), 2200, {.maxSpeed = 50}, false);
 pros::delay(500);
  // ! Score Middle
  chassis.turnToHeading(26 + 179+1.5+10, 1000, {}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(-13, 210.5), 1200, {.forwards = false, .minSpeed = 35}, false);

  
  middle_lift.retract();
  intake.setIntakeMultiplier(1.0, 1.0, 0.29);
  intake.setState(Intake::states::SCORING);
  intake.setMiddle(true);
  pros::delay(2000);
  intake.setIntakeMultiplier(1.0, 1.0, 1.0);

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
 middle_lift.extend();
 chassis.setPose({-(48 + 7.25 + 1.), 24, 90}, false);
 //chassis.moveToRelativePoint(Chassis::fromPolar(3, 0), 1200, {.maxSpeed = 100}, false);
  
  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(-25.35 , 24.0, 1000, {}, false);
  chassis.turnToHeading(179.0, 1000, {.minSpeed = 1}, false);
  chassis.moveToPoint(-25.25 , -100.0, 1100, {.maxSpeed = 50, .minSpeed = 20}, false);

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
  chassis.moveToPose(-37.5, 40, -90, 2000, {.lead = 0.8, .minSpeed = 10}, false);
  intake.setState(Intake::states::STORING);
  chassis.turnToHeading(-44-12-2, 1000, {.minSpeed = 1}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(16 * sqrt(2), -44-12-2), 1800, {.maxSpeed = 30}, false);

  //! Score Middle
  first_stage_lift.extend();
  chassis.turnToHeading(-44-2-2, 500, {}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(15, -44-12-2), 1200, {.maxSpeed = 80}, false);
  intake.setState(Intake::states::OUTTAKE);

  chassis.moveToRelativePoint(Chassis::fromPolar(-5.5, -44-12-2), 1200, {.maxSpeed = 127}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(6, -44-12-2), 1200, {.maxSpeed = 80}, false);

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
  ////chassis.moveToPoint(-27, 55, 1000, {.forwards = false, .minSpeed = 45});

  //pros::delay(400);
  //chassis.turnToHeading(188, 1000);
  //goal_descore_left.extend();
  
}
//does this work
void skills(bool is_red_team)
{
  
  intake.setState(Intake::states::STORING);
  chassis.setPose({54.5, 23.25, -90}, false);

  // UNLOAD GOAL 1
  matchload_unloader.extend();
  chassis.moveToPoint(24.-5.75+4, 23.25, 1000, {}, false);

  // Grab Blocks
  chassis.turnToHeading(-180, 1000, {}, false);
  chassis.moveToPoint(24.5-0.5-5.5 +4, -1000.0, 900, {.maxSpeed = 50, .minSpeed = 22}, false);
  chassis.turnToHeading(-187, 500, {}, false);
  chassis.moveToPoint(24.5-0.5-5.5 +4, -1000.0, 500, {.maxSpeed = 70, .minSpeed = 32}, false);
  chassis.turnToHeading(-180, 200, {}, false);
  chassis.moveToPoint(24.5-0.5-5.5 +4, -1000.0, 600, {.maxSpeed = 70, .minSpeed = 33}, false);

  chassis.moveToRelativePoint(0, 20-10, 1000, {.forwards = false}, false);
  intake.setState(Intake::states::STORING);
  matchload_unloader.retract();
  chassis.turnToHeading(-100+5, 600, {}, false);
  chassis.moveToRelativePoint(-13.5, 0, 700, {}, false);
  

  // ! RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({12, 34-10}, 8.0);
  pros::delay(650);

  auto left_goal_1 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_1.theta);

   // Go to loader 2
chassis.turnToHeading(-5, 600, {}, false);

 // !s RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({12, 34-10}, 8.0);
  pros::delay(650);

  auto left_goal_11 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_11.theta);

chassis.moveToPoint(7.35-2, 64, 2000, {.minSpeed = 10}, false);
   pros::delay(100);
   chassis.turnToPoint(9.1-2.25, 96, 700, {.minSpeed = 10}, false);//8.75
    chassis.moveToPoint(9.1-2.25, 96, 1300, {.minSpeed = 10}, false);
 chassis.turnToPoint(18.65, 113, 800, {.minSpeed = 30}, false);
 chassis.moveToPoint(18.7, 113, 900, {}, false);

   //* Score Loader 1
    chassis.turnToHeading(0, 600, {}, false);
  //intake.setState(Intake::states::STORING);  // STops bottom block from yeeting out
  middle_lift.extend();
  chassis.moveToRelativePoint(0.35, -23.25-.4, 1000, {.forwards = false, .maxSpeed = 68}, true);
  pros::delay(450);
  intake.setState(Intake::states::SCORING);
  intake.antiJam(true);
  pros::delay(2200);

//   // ! PIck up strays
   //intake.setState(Intake::states::STORING);
   //  chassis.moveToRelativePoint(0, 6, 800, {.minSpeed = 30}, false);
//   chassis.moveToRelativePoint(0, -6, 1000, {.forwards = false, .minSpeed = 30}, false);

//   // Score again
//   intake.setState(Intake::states::SCORING);
//   intake.antiJam(true);
//   pros::delay(2000);
  intake.setState(Intake::states::STORING);
  //* grabbing 2nd loader

  matchload_unloader.extend();
  pros::delay(200);
  chassis.moveToPoint(18.675, 1000, 1000, {.maxSpeed = 63, .minSpeed = 22}, true);
  pros::delay(200);
  middle_lift.retract();
  pros::delay(1500);
  //chassis.turnToHeading(-10, 500, {.minSpeed = 20});
  chassis.moveToPoint(18.7, 1000, 500, {.maxSpeed = 60, .minSpeed = 20}, false);
  chassis.turnToHeading(11, 500, {.minSpeed = 20});
  chassis.moveToPoint(18.7, 1000, 500, {.maxSpeed = 70, .minSpeed = 30}, false);
  chassis.turnToHeading(0, 500, {.minSpeed = 20});
  chassis.moveToPoint(18.6, 500, 500, {.maxSpeed = 70, .minSpeed = 35}, false);
  pros::delay(1000);
   // SCORE GOAL 2
//*scoring second loader
 //chassis.moveToRelativePoint(-0.5, -26, 1800, {.forwards = false, .maxSpeed = 70}, true);
 middle_lift.extend();
 chassis.moveToPoint(18.7, 89, 700, {.forwards = false, .maxSpeed = 70, .minSpeed = 30,}, false);

 pros::delay(300);
 pros::delay(600);
 intake.setState(Intake::states::SCORING);
 chassis.moveToRelativePoint(0, -26, 1000, {.forwards = false, .maxSpeed = 80}, true);
  intake.antiJam(true);
   pros::delay(2600);
//   chassis.moveToRelativePoint(0, 6, 800, {.minSpeed = 30}, false);
//   chassis.moveToRelativePoint(-.25, -6, 1000, {.forwards = false, .minSpeed = 30}, false);
//   intake.antiJam(true);
//   pros::delay(1800);
   matchload_unloader.retract();
   intake.setState(Intake::states::SCORING);
  // ! RESET TO LEFT GOAL 2
  auto left_goal_2 = chassis.getPose();
  chassis.setPose(24.8, 104, left_goal_2.theta);
//   matchload_unloader.extend();

//   chassis.moveToRelativePoint(0, 7, 800, {.minSpeed = 30}, false);
//   chassis.turnToPoint(46, 121, 1000, {.minSpeed = 10}, false);
//   chassis.moveToPoint(46, 121, 1000, {.maxSpeed = 72}, false);
//   chassis.turnToPoint(46, 20, 1000, {.minSpeed = 10}, false);
//   chassis.moveToPoint(46, 20, 2000, {.maxSpeed = 72}, true);
//   matchload_unloader.retract();
//   pros::delay(1000);
//   chassis.turnToPoint(80, 0, 1000, {.minSpeed = 10}, false);




  /*chassis.turnToHeading(0, 500, {.minSpeed = 20});
  intake.setState(Intake::states::STORING);
  matchload_unloader.extend();
  chassis.moveToPoint(125, 1000, 1000, {.maxSpeed = 65}, false);
  pros::delay(500);
  chassis.moveToPoint(125, 1000, 1000, {.maxSpeed = 65}, false);
  pros::delay(500);
  chassis.moveToPoint(125, 1000, 1000, {.maxSpeed = 65}, false);
  pros::delay(500);
  chassis.moveToPoint(125, 1000, 1000, {}, false);
  pros::delay(500);
  chassis.moveToPoint(125, 95, 1000, {.forwards = false}, false);
  intake.setState(Intake::states::SCORING);
  */
//   // GO TO GOAL 3
  
  chassis.moveToRelativePoint(-1.5, 8, 800, {.minSpeed = 10}, false);
//*   new clearing park stuff
  chassis.turnToPoint(38, 137,700, {.minSpeed = 10}, true);
  chassis.moveToPoint(38, 137, 800, {.minSpeed = 10}, false);

  chassis.turnToPoint(43.5, 143, 600, {.minSpeed = 10}, true);
  chassis.moveToPoint(43.5, 143, 700, {.minSpeed = 10}, false);

  chassis.turnToPoint(93, 157.5, 700, {.minSpeed = 10}, true);
  intake.setState(Intake::states::STORING);


//   left_motors.move_velocity(78);
//   right_motors.move_velocity(78);
//   pros::delay(360);
//   matchload_unloader.extend();
//   pros::delay(2000);
//   matchload_unloader.retract();
//   do{
//     front_distance.update(Point(chassis.getPose().x, chassis.getPose().y), chassis.getPose().theta);
//     pros::delay(20);
//   } while (front_distance.get_distance_reading() > 40);
//   left_motors.brake();
//   right_motors.brake();


  chassis.moveToRelativePoint(100, 16, 2000, {.forwards = true, .maxSpeed = 75, .minSpeed = 35}, true);
  pros::delay(360);
  matchload_unloader.extend();
  pros::delay(2000);
  matchload_unloader.retract();
  chassis.moveToRelativePoint(15, 0, 800, {.forwards = true, .maxSpeed = 90, .minSpeed = 38}, true);
   // !s RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({98, 140}, 8.0);
  pros::delay(800);

  auto right_goal_1 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_goal_1.theta);

    

chassis.moveToRelativePoint(5, 0, 1500, {.forwards = true, .maxSpeed = 90, .minSpeed = 38}, true);

//*correcting after the park clear
 // !s RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({103, 140}, 8.0);
  pros::delay(900);

  auto right_goal_11 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_goal_11.theta);

pros::delay(400);

chassis.turnToPoint(132.5+.1, 121, 1000, {.minSpeed = 10}, false);
chassis.moveToPoint(132.5+.1, 121, 1000, {.minSpeed = 10}, false);
chassis.turnToHeading(0, 800, {.minSpeed = 20});
 chassis.moveToRelativePoint(0.1, -23, 1000, {.forwards = false, .maxSpeed = 74}, false);

 //* scoring the few park blocks
 intake.setState(Intake::states::SCORING);
 pros::delay(1500);
 intake.setState(Intake::states::STORING);
 matchload_unloader.extend();

 //* gathering 3rd loader blocks
  chassis.moveToRelativePoint(0, 1000, 700, {.forwards = true, .maxSpeed = 72}, false);
 pros::delay(600);
  chassis.moveToRelativePoint(0, 1000, 700, {.forwards = true, .maxSpeed = 72}, false);
 pros::delay(700);
  chassis.moveToRelativePoint(0, 1000, 700, {.forwards = true, .maxSpeed = 72}, false);
 pros::delay(800);
 chassis.moveToRelativePoint(0.08, -40, 850, {.forwards = false, .maxSpeed = 65}, false);
 pros::delay(500);

 //*Score 3rd loaders
intake.setState(Intake::states::SCORING);
pros::delay(500);
intake.setState(Intake::states::OUTTAKE);
pros::delay(100);
intake.setState(Intake::states::SCORING);
pros::delay(800);
matchload_unloader.retract();










//   // GO TO GOAL 3 b
//   chassis.turnToPoint(112+3, 107, 1000, {.minSpeed = 10}, true);

//   // Loader 3
//   chassis.moveToPoint(112+3, 107, 2000, {.minSpeed = 10}, false);

//   chassis.turnToHeading(0, 1200, {}, false);
//   matchload_unloader.extend();
//   pros::delay(800);
//   chassis.moveToPoint(114.3, 1000, 2000, {.maxSpeed = 60, .minSpeed = 23}, false);
//   chassis.turnToHeading(10, 500, {.minSpeed = 20});
//   chassis.turnToHeading(-10, 600, {.minSpeed = 20});
//   chassis.turnToHeading(0, 800, {.minSpeed = 21});

//   // GO to other side
//   chassis.moveToRelativePoint(0.3, -12, 1500, {.forwards = false}, false);
//   matchload_unloader.retract();
//   chassis.turnToHeading(90, 1800, {.minSpeed = 10}, false);
//   chassis.moveToRelativePoint(12+1.5, 0, 2000, {}, false);

//   // ! RESET TO LEFT GOAL 2 using mcl (without back sensor)
  mcl_localization.reset_particles({134, 120.5}, 8.0);
  pros::delay(750);

  auto left_goal_3 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_3.theta);

  

//   chassis.moveToPoint(142, 120, 1000, {.minSpeed = 10}, false);
//   chassis.turnToHeading(2, 1000, {}, false);
//   //* driving across the alley to home land
chassis.moveToRelativePoint(0, 15, 1000, {.forwards = true, .maxSpeed = 72}, false);
chassis.turnToHeading(90, 1000, {}, false);
chassis.moveToRelativePoint(15, 0, 1000, {.forwards = true, .maxSpeed = 72}, false);
 // !* RESET TO Right GOAL 1 using mcl
  mcl_localization.reset_particles({140, 140}, 8.0);
  pros::delay(700);

  auto right_goal_2 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_goal_2.theta);

chassis.turnToHeading(180, 700, {}, false);

 // * RESET TO Right GOAL 2 using mcl
  mcl_localization.reset_particles({140, 140}, 8.0);
  pros::delay(700);

  auto right_goal_22 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_goal_22.theta);


chassis.moveToPoint(146, 30, 2000, {.forwards = true, .maxSpeed = 75}, false);
chassis.moveToPoint(121, 30, 2000, {.forwards = true, .maxSpeed = 75}, false);
chassis.turnToHeading(180, 1000, {}, false);


//   //chassis.moveToRelativePoint(3, -100, 2000, {.forwards = false, .maxSpeed = 69}, false);
//   chassis.moveToPoint(127, 42, 2000, {.forwards = false, .minSpeed = 10}, false);
//   chassis.turnToPoint(180, 0, 1000, {.minSpeed = 10}, false);
  
  //chassis.turnToPoint(137.2, 30, 1000, {.minSpeed = 10}, false);
  //chassis.moveToPoint(137.2, 30, 1500, {.maxSpeed = 66}, false);
//   // Drive down the alley
//   chassis.turnToPoint(141, 48, 1500, {.minSpeed = 20}, false);
//   chassis.moveToPoint(141, 48, 1500, {.minSpeed = 10}, false);

//   // Start aligning to score
//   chassis.turnToPoint(124.5, 24, 1500, {.minSpeed = 50}, false);
//   chassis.moveToPoint(124.5, 24, 1500, {}, false);
//   chassis.turnToHeading(180, 1000, {}, false);

//   // SCORE GOAL 3
//   mcl_localization.reset_particles({123, 24}, 8.0);
//   pros::delay(500);

//   auto right_goal_1 = chassis.getPose();
//   chassis.setPose(
//       mcl_localization.get_point_estimate().x,
//       mcl_localization.get_point_estimate().y,
//       right_goal_1.theta);

//   chassis.moveToRelativePoint(1, 38, 1800, {.forwards = false, .maxSpeed = 100}, false);
//   //chassis.moveToPoint(124 -0.5, 24 +38, 1500, {.forwards = false}, false);
//   intake.setState(Intake::states::SCORING);
//   intake.antiJam(true);
//   pros::delay(2000);
//   intake.antiJam(true);
//   pros::delay(1500);
//   intake.setState(Intake::states::STORING);

//   // LOADER 4
//   matchload_unloader.extend();
//   chassis.moveToRelativePoint(-1.6, -18.0, 800, {.maxSpeed = 90, .minSpeed = 40}, false);
//   chassis.moveToRelativePoint(-1.6, -1000, 2500, {.maxSpeed = 50, .minSpeed = 20}, false);


//   // Score Goal 4
//   chassis.moveToRelativePoint(3.7, 38, 1800, {.forwards = false, .maxSpeed = 100}, false);
//   //hassis.moveToPoint(124 -0.5, 24 +38, 1500, {.forwards = false}, false);
//   intake.setState(Intake::states::SCORING);
//   intake.antiJam(true);
//   pros::delay(900);
//   intake.antiJam(true);
//   pros::delay(1200);
//   intake.setState(Intake::states::STORING);

//   // Park
//   matchload_unloader.retract();
//   auto right_goal_2 = chassis.getPose(false);
//   chassis.setPose(120, 40, right_goal_2.theta);
//   chassis.moveToPoint(108, 14-0.5, 2000, {}, false);
//   chassis.turnToHeading(-100, 1000);
//   chassis.moveToRelativePoint(Chassis::fromPolar(50, -100), 2000, {.minSpeed = 80}, false);
//   chassis.moveToRelativePoint(
//       Chassis::fromPolar(-5, -100), 2000, {.forwards = false, .minSpeed = 60}, false);
}

void rightSideFinals(bool is_red_team)
{
  chassis.setPose({144 - 46, 24, 0});
   chassis.moveToPoint(144 - 46, 38, 2000, {.forwards = true}, false);
//   //* Pick Up 3 Cluster
//   intake.setState(Intake::states::STORING);
//   chassis.moveToPoint(144 - 46, 38, 2000, {.maxSpeed = 100, .minSpeed = 20}, false);
//   chassis.swingToHeading(
//       40, lemlib::DriveSide::RIGHT, 2000, {.maxSpeed = 60, .minSpeed = 20}, false);

//   // ! Go to 2 Cluster
//   chassis.moveToRelativePose(
//       lemlib::Pose{5, 0, 90 - 40} + Chassis::fromPolar(31, 40, false),
//       2000,
//       {.lead = 0.6, .maxSpeed = 80},
//       false);
//   matchload_unloader.extend();
//   pros::delay(300);

//   // ? Go to Goal
//   chassis.moveToRelativePose({0, -48, 180}, 2000, {.forwards = false, .minSpeed = 30}, false);
//   chassis.moveToRelativePose(
//       {16, 36, -95}, 3000, {.forwards = false, .lead = 0.9, .minSpeed = 10}, false);

//   // * Score
//   intake.setState(Intake::states::SCORING);
//   matchload_unloader.retract();
//   pros::delay(2000);

//   // * Go to loader
//   intake.setState(Intake::states::STORING);
//   chassis.moveToRelativePoint(0, -18, 2000, {}, false);
//   matchload_unloader.extend();
//   chassis.moveToRelativePoint(0, -100, 1200, {.maxSpeed = 40, .minSpeed = 20}, false);
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