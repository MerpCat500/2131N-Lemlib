#include "autonomous.hpp"

#include <cmath>

#include "2131N/robot-config.hpp"
#include "2131N/systems/chassis.hpp"
#include "2131N/systems/intake.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"


void debug(bool is_red_team) { chassis.setPose({0, 0, 0}, false); }
//* right side awp
void leftSideAwp(bool is_red_team)
{
middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({87.75, 24, 90}, false);
  
  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(50);  // Wait for the loader to extend before moving used to be 200
  chassis.moveToPoint(115-1, 24.0, 1110, {.maxSpeed = 125, .minSpeed = 78}, false);
  chassis.turnToHeading(180.0, 640, {.maxSpeed = 60, .minSpeed = 47}, false);
    chassis.moveToPoint(115-1, -100.0, 1225, {.maxSpeed = 60, .minSpeed = 52}, false);

   // ! Attempt to score
   auto after_loader = chassis.getPose();
   chassis.moveToPoint(after_loader.x +0.25, after_loader.y + 36.0, 510, {.forwards = false, .maxSpeed = 125, .minSpeed = 123});
   pros::delay(0);
   chassis.moveToRelativePoint(Chassis::fromPolar(0, -4), 10, {.forwards = false, .minSpeed = 40}, true);
   intake.setState(Intake::states::SCORING);
   pros::delay(300);
  //intake.setState(Intake::states::OUTTAKE);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.5), 100, {.forwards = false, .minSpeed = 100}, true);
   pros::delay(100);
  intake.setState(Intake::states::SCORING);
   pros::delay(1100);
 
   matchload_unloader.retract();
   chassis.cancelMotion();

   // * Reset to the goal
   auto left_goal = chassis.getPose();
   chassis.setPose({119.75, 48 - 7.25 + 2.5, left_goal.theta});

    // ? Grab Middle
   intake.setState(Intake::states::STORING);
   chassis.moveToPose(102, 36.5, -70, 1200, {.lead = 0.9, .minSpeed = 42}, false);
   //intake.setState(Intake::states::STORING);
   chassis.turnToPoint(98, 47.25, 400, {.minSpeed = 45});
   chassis.moveToPoint(98, 47.25, 1000, {.maxSpeed = 125, .minSpeed = 55}, true);
   pros::delay(200);
   matchload_unloader.extend();
   pros::delay(450);
   matchload_unloader.retract();

   chassis.turnToPoint(46, 6.5, 500, {.minSpeed = 30});
   chassis.moveToPoint(46, 46.5, 1300, {.maxSpeed = 125, .minSpeed = 55}, true);
   pros::delay(600);
   matchload_unloader.extend();
   pros::delay(300);
   //matchload_unloader.retract();

   chassis.moveToPoint(49, 46.25, 160, {.forwards = false, .maxSpeed = 90, .minSpeed = 50}, false);

   // * line up to score
   chassis.turnToPoint(58+4.7, 58, 260, {.forwards = false, .minSpeed = 45});
   chassis.moveToPoint(58+4.7, 57.5, 700, {.forwards = false,  .minSpeed = 55}, false);


   // ! Score Middle
   middle_lift.retract();
   intake.setIntakeMultiplier(1.0, 1.0, 0.29);
   intake.setState(Intake::states::SCORING);
   intake.setMiddle(true);
   pros::delay(700);
   intake.setIntakeMultiplier(1.0, 1.0, 1.0);
   intake.setState(Intake::states::STORING);

   // ? go to other loader 

   matchload_unloader.extend();

   chassis.turnToPoint(24+9.5, 24, 500, { .minSpeed = 40});
   chassis.moveToPoint(24+9.5, 24, 1100, { .maxSpeed = 90, .minSpeed = 85}, false);

//   // * loader clear 


   chassis.turnToHeading(180.0, 470, {.minSpeed = 45}, false);
  

    chassis.moveToPoint(24+9.75, -100.0, 760, {.maxSpeed = 68, .minSpeed = 59}, false);

//   // ! Attempt to score
// //   auto after_loader2 = chassis.getPose();
// //   chassis.moveToPoint(after_loader2.x -1.2, after_loader2.y + 36.0, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 90});
   middle_lift.extend();
   chassis.moveToRelativePoint(0.2, 48, 545, {.forwards = false, .minSpeed = 125}, false);
  
//   //middle_lift.extend();
  chassis.moveToRelativePoint(0, 15, 200, {.forwards = false, .minSpeed = 41}, true);
  
  intake.setState(Intake::states::SCORING);
   pros::delay(800);
   intake.setState(Intake::states::SCORING);
   pros::delay(300);
   matchload_unloader.retract();
//  //intake.setState(Intake::states::OUTTAKE);
   pros::delay(100);
  intake.setState(Intake::states::SCORING);
   pros::delay(1600);


 









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
  chassis.moveToPoint(28.25 , 24.0, 2000, {.maxSpeed = 80, .minSpeed = 35}, false);
  chassis.turnToHeading(-180.0, 1000, {}, false);
  chassis.moveToPoint(28.25, -100.0, 1000, {.maxSpeed = 80, .minSpeed = 35}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(after_loader.x , after_loader.y + 36.0, 1600, {.forwards = false, .maxSpeed = 75, .minSpeed = 30});
  pros::delay(600);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.23), 600, {.forwards = false, .minSpeed = 50}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(300);
 intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
 intake.setState(Intake::states::SCORING);
  pros::delay(1300);
 
  matchload_unloader.retract();
  chassis.cancelMotion();

  // * Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24+.5,48 - 7.25 + 2.5, left_goal.theta});

  // ? Grab Middle
  intake.setState(Intake::states::STORING);
  chassis.moveToPose(43, 36.25, 90, 2000, {.lead = 0.9, .minSpeed = 20}, false);
  intake.setState(Intake::states::STORING);
  chassis.turnToHeading(46, 1000, {.minSpeed = 10}, false);
  chassis.moveToRelativePoint(Chassis::fromPolar(14 * sqrt(2), 46), 1000, {.maxSpeed = 50}, true);
  pros::delay(200);
  matchload_unloader.extend();
  
 pros::delay(1200);
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
  middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({87.75, 24, 90}, false);
  
  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(50);  // Wait for the loader to extend before moving used to be 200
  chassis.moveToPoint(115.25, 24.0, 1500, {.maxSpeed = 80, .minSpeed = 50}, false);
  chassis.turnToHeading(180.0, 640, {.maxSpeed = 80, .minSpeed = 50}, false);
    chassis.moveToPoint(115.25, -100.0, 1000, {.maxSpeed = 65, .minSpeed = 30}, false);

   // ! Attempt to score
   auto after_loader = chassis.getPose();
   chassis.moveToPoint(after_loader.x +0.25, after_loader.y + 36.0, 1000, {.forwards = false, .maxSpeed = 125, .minSpeed = 60});
   pros::delay(0);
   chassis.moveToRelativePoint(Chassis::fromPolar(0, -4), 200, {.forwards = false, .minSpeed = 40}, true);
   intake.setState(Intake::states::SCORING);
   pros::delay(300);
  //intake.setState(Intake::states::OUTTAKE);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.5), 200, {.forwards = false, .minSpeed = 100}, true);
   pros::delay(100);
  intake.setState(Intake::states::SCORING);
   pros::delay(1100);
 
   matchload_unloader.retract();
   chassis.cancelMotion();

   // * Reset to the goal
   auto left_goal = chassis.getPose();
   chassis.setPose({119.75, 48 - 7.25 + 2.5, left_goal.theta});

    // ? Grab Middle
   intake.setState(Intake::states::STORING);
   chassis.moveToPose(102, 36.5, -70, 1200, {.lead = 0.9, .minSpeed = 42}, false);
   //intake.setState(Intake::states::STORING);
   chassis.turnToPoint(98, 47.25, 400, {.minSpeed = 45});
   chassis.moveToPoint(96.25, 47.25, 1000, {.maxSpeed = 125, .minSpeed = 55}, true);
   pros::delay(200);
   matchload_unloader.extend();
   pros::delay(450);
   matchload_unloader.retract();
   
   chassis.turnToHeading(-50, 640, {.maxSpeed = 60, .minSpeed = 47}, false);
   pros::delay(125);
   chassis.moveToRelativePoint(-20, 20, 1000, {.forwards = true}, false);
   chassis.cancelMotion();
   intake.setState(Intake::states::OUTTAKE);
   pros::delay(1500);
   chassis.moveToRelativePoint(21.95, -20, 1000, {.forwards = false}, false);
   chassis.turnToHeading(0, 640, {.maxSpeed = 60, .minSpeed = 47}, false);
   goal_descore_right.retract();
   chassis.moveToRelativePoint(0, 25, 1000, {.forwards = true}, false);
  /*
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
  */
  
}
//does this work
void skills(bool is_red_team)
{
   intake.setState(Intake::states::STORING);
  chassis.setPose({54.5, 23.25, -90}, false);

  // UNLOAD GOAL 1
  matchload_unloader.extend();
  chassis.moveToPoint(24.-5.75+5, 23.25, 1000, {}, false);

  // Grab Blocks
  chassis.turnToHeading(-180, 1000, {}, false);
  chassis.moveToPoint(24.5-0.5-5 +4.5, -1000.0, 900, {.maxSpeed = 62, .minSpeed = 22}, false);
  chassis.turnToHeading(-187, 500, {}, false);
  chassis.moveToPoint(24.5-0.5-5 +5, -1000.0, 500, {.maxSpeed = 80, .minSpeed = 32}, false);
  chassis.turnToHeading(-180, 500, {}, false);
  chassis.moveToPoint(24.5-0.5-5 +5, -1000.0, 800, {.maxSpeed = 75, .minSpeed = 33}, false);

  chassis.moveToRelativePoint(0, 20-10, 1000, {.forwards = false}, false);
  intake.setState(Intake::states::STORING);
  matchload_unloader.retract();
  chassis.turnToHeading(-100+5, 600, {}, false);
  chassis.moveToRelativePoint(-16.5, 0, 710, {}, false);
  

  // ! RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({12, 34-10}, 8.0);
  pros::delay(650);

  auto left_goal_1 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_1.theta);

   // Go to loader 2
chassis.turnToHeading(-7, 600, {}, false);

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
    chassis.turnToPoint(9.1-2.25, 96, 700, {.minSpeed = 10}, false);
     chassis.moveToPoint(9.1-2.25, 96, 1300, {.minSpeed = 10}, false);
  chassis.turnToPoint(19.85, 113-3, 800, {.minSpeed = 30}, false);
  chassis.moveToPoint(20.825, 113-3, 900, {}, false);

    //* Score Loader 1
     chassis.turnToHeading(-0.2, 600, {}, false);

  middle_lift.extend();

  chassis.moveToPoint(19.5, 88, 1000, {.forwards = false, .maxSpeed = 61.5, .minSpeed = 22}, true);
  pros::delay(450);
  intake.setState(Intake::states::SCORING);
  pros::delay(600);
  intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
  intake.setState(Intake::states::SCORING);
  pros::delay(1400);
  intake.setState(Intake::states::STORING);
  //* grabbing 2nd loader
 
  matchload_unloader.extend();
  pros::delay(200);
  chassis.moveToPoint(18.35, 1000, 1000, {.maxSpeed = 61.5, .minSpeed = 22}, true);
  pros::delay(200);
  middle_lift.retract();
  pros::delay(1500);

  chassis.moveToPoint(18.15, 1000, 500, {.maxSpeed = 60, .minSpeed = 20}, false);
  chassis.turnToHeading(11, 500, {.minSpeed = 20});
  chassis.moveToPoint(18.6, 1000, 500, {.maxSpeed = 70, .minSpeed = 30}, false);
  chassis.turnToHeading(0, 500, {.minSpeed = 20});
  chassis.moveToPoint(18.6, 500, 500, {.maxSpeed = 70, .minSpeed = 35}, false);
  pros::delay(1000);
   // SCORE GOAL 2
//*scoring second loader
 
 middle_lift.extend();
 chassis.moveToPoint(18.75, 89, 700, {.forwards = false, .maxSpeed = 70, .minSpeed = 30,}, false);

 pros::delay(300);
 pros::delay(600);
 intake.setState(Intake::states::SCORING);
 chassis.moveToRelativePoint(0, -26, 1000, {.forwards = false, .maxSpeed = 80}, true);
  intake.antiJam(true);
   pros::delay(2800);

   matchload_unloader.retract();
   intake.setState(Intake::states::SCORING);
  // ! RESET TO LEFT GOAL 2
  auto left_goal_2 = chassis.getPose();
  chassis.setPose(24.8, 104, left_goal_2.theta);
  intake.setState(Intake::states::STORING);
  //* moving off goal 1
  chassis.moveToRelativePoint(0, 8, 800, {.minSpeed = 10}, false);

  ///chassis.turnToHeading(90, 500, {.minSpeed = 10});
  middle_lift.retract();
  //*clearing park
 // chassis.moveToPoint(110.5, 119, 2500, {.maxSpeed = 77, .minSpeed = 35}, false);
  chassis.turnToHeading(40, 500, {.minSpeed = 10});
  chassis.moveToPoint(46, 140, 700, {.forwards = true, .maxSpeed = 70, .minSpeed = 30,}, false);
  chassis.moveToPoint(49, 155, 800, {.forwards = true, .maxSpeed = 70, .minSpeed = 30,}, false);
  chassis.turnToHeading(40, 500, {.minSpeed = 10});
  
  chassis.moveToRelativePoint(100, 8, 800, {.maxSpeed = 75, .minSpeed = 37}, true);
  pros::delay(200);
  matchload_unloader.retract();
  pros::delay(1200);
  matchload_unloader.extend();
  pros::delay(200000000);
  //* emptying loader 3
  chassis.turnToHeading(0, 500, {.minSpeed = 10});
  matchload_unloader.extend();
  pros::delay(200);
  
  chassis.moveToPoint(111.35, 1200, 900, {.maxSpeed = 62, .minSpeed = 35}, false);
  pros::delay(600);
  chassis.moveToPoint(111.35, 1200, 900, {.maxSpeed = 80, .minSpeed = 35}, false);
  pros::delay(600);
  chassis.moveToPoint(111.35, 1200, 900, {.maxSpeed = 70, .minSpeed = 35}, false);
  pros::delay(700);

  //* moving off loader
  chassis.moveToRelativePoint(0, -15, 1000, {.forwards = false, .minSpeed = 10}, false);
  matchload_unloader.retract();
  chassis.turnToHeading(90, 700, {.minSpeed = 10});
  chassis.moveToRelativePoint(32, 0, 1000, {.minSpeed = 10}, false);
     // !s RESET TO RIGHT WALL 1 using mcl
  mcl_localization.reset_particles({132, 103}, 8.0);
  pros::delay(750);

  auto right_wall_11 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_wall_11.theta);


  chassis.turnToHeading(150, 500, {.minSpeed = 10});

   // !s RESET TO RIGHT WALL 1 using mcl
  mcl_localization.reset_particles({124, 103}, 8.0);
  pros::delay(750);

  auto right_wall_2 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_wall_2.theta);

  //* driving down the alley to home lands
  
 
  chassis.moveToRelativePoint(12, -110, 3000, {.forwards = true,.maxSpeed = 81, .minSpeed = 10}, false);

   // !s RESET TO RIGHT WALL 2 using mcl
  mcl_localization.reset_particles({126, 10}, 8.0);
  pros::delay(750);

  auto right_wall_3 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_wall_3.theta);

  chassis.moveToRelativePoint(-14, 21, 1000, {.forwards = false, .minSpeed = 10}, false);
  chassis.turnToHeading(180, 700, {.minSpeed = 10});
  middle_lift.extend();

  //* scoring goal 3

  chassis.moveToPoint(120, 65, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 35}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(600);
  intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
  intake.setState(Intake::states::SCORING);
  pros::delay(1300);

  matchload_unloader.extend();
  intake.setState(Intake::states::STORING);

  //*grabbing 4th loader
  chassis.moveToPoint(124, -100, 1000, {.forwards = true, .maxSpeed = 61, .minSpeed = 35}, false);
  pros::delay(500);
  chassis.moveToPoint(123.7, -100, 1000, {.forwards = true, .maxSpeed = 75, .minSpeed = 35}, false);
  pros::delay(500);
  chassis.moveToPoint(123.5, -100, 1000, {.forwards = true, .maxSpeed = 72, .minSpeed = 35}, false);
  pros::delay(600);

  //* scoring 4th loader
  chassis.moveToRelativePoint(-0.25, 66, 1000, {.forwards = false, .maxSpeed = 65}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(600);
   intake.setState(Intake::states::SCORING);
  pros::delay(200);
   intake.setState(Intake::states::SCORING);
  pros::delay(900);
  matchload_unloader.retract();

  // ! RESET TO Right GOAL 2
  pros::delay(300);
  auto right_goal_4 = chassis.getPose();
  chassis.setPose(125.2, 45, right_goal_4.theta);

  //* moving off goal
  chassis.moveToPoint(102, -30, 800, {.forwards = true, .maxSpeed = 61.5, .minSpeed = 35}, false);
  chassis.moveToPoint(102, -50, 850, {.forwards = true, .maxSpeed = 61.5, .minSpeed = 35}, false);
  chassis.turnToHeading(-130, 700, {.minSpeed = 10});

  //* clearing park
  intake.setState(Intake::states::STORING);
 chassis.moveToRelativePoint(-65, -20, 1000, {.forwards = true, .maxSpeed = 71}, true);
 pros::delay(450);
 matchload_unloader.extend();
 pros::delay(350);
 chassis.moveToRelativePoint(-52, -5, 1100, {.forwards = true, .minSpeed = 35}, false);
 matchload_unloader.retract();
 chassis.moveToRelativePoint(3, 0, 1000, {.forwards = false, .minSpeed = 35}, false);
  

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
  chassis.setPose({54.5, 23.25, -90}, false);

  // UNLOAD GOAL 1
  matchload_unloader.extend();
  chassis.moveToPoint(24.-5.75+5, 23.25, 1000, {}, false);

  // Grab Blocks
  chassis.turnToHeading(-180, 1000, {}, false);
  chassis.moveToPoint(24.5-0.5-5 +4.5, -1000.0, 900, {.maxSpeed = 62, .minSpeed = 22}, false);
  chassis.turnToHeading(-187, 500, {}, false);
  chassis.moveToPoint(24.5-0.5-5 +5, -1000.0, 500, {.maxSpeed = 80, .minSpeed = 32}, false);
  chassis.turnToHeading(-180, 500, {}, false);
  chassis.moveToPoint(24.5-0.5-5 +5, -1000.0, 800, {.maxSpeed = 75, .minSpeed = 33}, false);

  chassis.moveToRelativePoint(0, 20-10, 1000, {.forwards = false}, false);
  intake.setState(Intake::states::STORING);
  matchload_unloader.retract();
  chassis.turnToHeading(-100+5, 600, {}, false);
  chassis.moveToRelativePoint(-16.5, 0, 710, {}, false);
  

  // ! RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({12, 34-10}, 8.0);
  pros::delay(650);

  auto left_goal_1 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_1.theta);

   // Go to loader 2
chassis.turnToHeading(-7, 600, {}, false);

 // !s RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({12, 34-10}, 8.0);
  pros::delay(650);

  auto left_goal_11 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_11.theta);
  //chassis.turnToHeading(-31, 400, {}, false);
  //chassis.moveToRelativePoint(-23.5, 200, 2500, {.forwards = true, .maxSpeed = 85}, false);
 chassis.moveToPoint(7.35-2, 64, 2000, {.minSpeed = 10}, false);
  /*
// !s RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({12, 134}, 8.0);
  pros::delay(650);

  auto left_wall_11 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_wall_11.theta);

  chassis.moveToRelativePoint(0, 2, 1000, {.forwards = false, .maxSpeed = 72}, false);

  // !s RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({12, 134}, 8.0);
  pros::delay(650);

  auto left_wall_111 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_wall_111.theta);
      */
 pros::delay(100);
    chassis.turnToPoint(9.1-2.25, 96, 700, {.minSpeed = 10}, false);
     chassis.moveToPoint(9.1-2.25, 96, 1300, {.minSpeed = 10}, false);
  chassis.turnToPoint(19.85, 113-3, 800, {.minSpeed = 30}, false);
  chassis.moveToPoint(20.825, 113-3, 900, {}, false);

    //* Score Loader 1
     chassis.turnToHeading(-0.2, 600, {}, false);

  middle_lift.extend();
  //chassis.moveToPoint(34, -50, 1000, {.forwards = false, .maxSpeed = 61.5, .minSpeed = 22}, false);
  //chassis.moveToRelativePoint(0, -5, 1000, {.forwards = false, .maxSpeed = 72}, true);
  chassis.moveToPoint(19.5, 88, 1000, {.forwards = false, .maxSpeed = 61.5, .minSpeed = 22}, true);
  pros::delay(450);
  intake.setState(Intake::states::SCORING);
  pros::delay(600);
  intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
  intake.setState(Intake::states::SCORING);
  pros::delay(1400);
  intake.setState(Intake::states::STORING);
  //* grabbing 2nd loader
 
  matchload_unloader.extend();
  pros::delay(200);
  chassis.moveToPoint(18.35, 1000, 1000, {.maxSpeed = 61.5, .minSpeed = 22}, true);
  pros::delay(200);
  middle_lift.retract();
  pros::delay(1500);

  chassis.moveToPoint(18.35, 1000, 500, {.maxSpeed = 60, .minSpeed = 20}, false);
  chassis.turnToHeading(11, 500, {.minSpeed = 20});
  chassis.moveToPoint(18.6, 1000, 500, {.maxSpeed = 70, .minSpeed = 30}, false);
  chassis.turnToHeading(0, 500, {.minSpeed = 20});
  chassis.moveToPoint(18.6, 500, 500, {.maxSpeed = 70, .minSpeed = 35}, false);
  pros::delay(1000);
   // SCORE GOAL 2
//*scoring second loader
 
 middle_lift.extend();
 chassis.moveToPoint(18.75, 89, 700, {.forwards = false, .maxSpeed = 70, .minSpeed = 30,}, false);

 pros::delay(300);
 pros::delay(600);
 intake.setState(Intake::states::SCORING);
 chassis.moveToRelativePoint(0, -26, 1000, {.forwards = false, .maxSpeed = 80}, true);
  intake.antiJam(true);
   pros::delay(2800);

   matchload_unloader.retract();
   intake.setState(Intake::states::SCORING);
  // ! RESET TO LEFT GOAL 2
  auto left_goal_2 = chassis.getPose();
  chassis.setPose(24.8, 104, left_goal_2.theta);
  intake.setState(Intake::states::STORING);
  //* moving off goal 1
  chassis.moveToRelativePoint(0, 8, 800, {.minSpeed = 10}, false);

  chassis.turnToHeading(90, 500, {.minSpeed = 10});
  middle_lift.retract();
  chassis.moveToPoint(110.5, 119, 2500, {.maxSpeed = 77, .minSpeed = 35}, false);
  

  //* emptying loader 3
  chassis.turnToHeading(0, 500, {.minSpeed = 10});
  matchload_unloader.extend();
  pros::delay(200);
  
  chassis.moveToPoint(111.35, 1200, 900, {.maxSpeed = 62, .minSpeed = 35}, false);
  pros::delay(600);
  chassis.moveToPoint(111.35, 1200, 900, {.maxSpeed = 80, .minSpeed = 35}, false);
  pros::delay(600);
  chassis.moveToPoint(111.35, 1200, 900, {.maxSpeed = 70, .minSpeed = 35}, false);
  pros::delay(700);

  //* moving off loader
  chassis.moveToRelativePoint(0, -15, 1000, {.forwards = false, .minSpeed = 10}, false);
  matchload_unloader.retract();
  chassis.turnToHeading(90, 700, {.minSpeed = 10});
  chassis.moveToRelativePoint(32, 0, 1000, {.minSpeed = 10}, false);
     // !s RESET TO RIGHT WALL 1 using mcl
  mcl_localization.reset_particles({132, 103}, 8.0);
  pros::delay(750);

  auto right_wall_11 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_wall_11.theta);


  chassis.turnToHeading(150, 500, {.minSpeed = 10});

   // !s RESET TO RIGHT WALL 1 using mcl
  mcl_localization.reset_particles({124, 103}, 8.0);
  pros::delay(750);

  auto right_wall_2 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_wall_2.theta);

  //* driving down the alley to home lands
  
 
  chassis.moveToRelativePoint(12, -110, 3000, {.forwards = true,.maxSpeed = 81, .minSpeed = 10}, false);

   // !s RESET TO RIGHT WALL 2 using mcl
  mcl_localization.reset_particles({126, 10}, 8.0);
  pros::delay(750);

  auto right_wall_3 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      right_wall_3.theta);

  chassis.moveToRelativePoint(-14, 21, 1000, {.forwards = false, .minSpeed = 10}, false);
  chassis.turnToHeading(180, 700, {.minSpeed = 10});
  middle_lift.extend();

  //* scoring goal 3

  chassis.moveToPoint(120, 65, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 35}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(600);
  intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
  intake.setState(Intake::states::SCORING);
  pros::delay(1300);

  matchload_unloader.extend();
  intake.setState(Intake::states::STORING);

  //*grabbing 4th loader
  chassis.moveToPoint(124.25, -100, 1000, {.forwards = true, .maxSpeed = 60.5, .minSpeed = 32}, false);
  pros::delay(500);
  chassis.moveToPoint(123.7, -100, 1000, {.forwards = true, .maxSpeed = 75, .minSpeed = 35}, false);
  pros::delay(500);
  chassis.moveToPoint(123.5, -100, 1000, {.forwards = true, .maxSpeed = 72, .minSpeed = 35}, false);
  pros::delay(600);

  //* scoring 4th loader
  chassis.moveToRelativePoint(-0.25, 66, 1000, {.forwards = false, .maxSpeed = 65}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(600);
   intake.setState(Intake::states::SCORING);
  pros::delay(200);
   intake.setState(Intake::states::SCORING);
  pros::delay(900);
  matchload_unloader.retract();

  // ! RESET TO Right GOAL 2
  pros::delay(300);
  auto right_goal_4 = chassis.getPose();
  chassis.setPose(125.2, 45, right_goal_4.theta);

  //* moving off goal
  chassis.moveToPoint(102, -30, 800, {.forwards = true, .maxSpeed = 61.5, .minSpeed = 35}, false);
  chassis.moveToPoint(102, -50, 850, {.forwards = true, .maxSpeed = 61.5, .minSpeed = 35}, false);
  chassis.turnToHeading(-130, 700, {.minSpeed = 10});

  //* clearing park
  intake.setState(Intake::states::STORING);
 chassis.moveToRelativePoint(-65, -20, 1000, {.forwards = true, .maxSpeed = 71}, true);
 pros::delay(450);
 matchload_unloader.extend();
 pros::delay(350);
 chassis.moveToRelativePoint(-52, -5, 1100, {.forwards = true, .minSpeed = 35}, false);
 matchload_unloader.retract();
 chassis.moveToRelativePoint(3, 0, 1000, {.forwards = false, .minSpeed = 35}, false);
  


}
