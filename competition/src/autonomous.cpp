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
  
  // ! De-score loader
  matchload_unloader.extend();
  pros::delay(50);  // Wait for the loader to extend before moving used to be 200
  chassis.moveToPoint(115-1.75, 24.0, 1110, {.maxSpeed = 125, .minSpeed = 80}, false);
  chassis.turnToHeading(180.0, 640, {.maxSpeed = 70, .minSpeed = 50}, false);
    chassis.moveToPoint(115-1.75, -100.0, 1195, {.maxSpeed = 66, .minSpeed = 42}, false);

   // ! Attempt to score
   auto after_loader = chassis.getPose();
   chassis.moveToPoint(after_loader.x , after_loader.y + 38.0, 650, {.forwards = false, .maxSpeed = 125, .minSpeed = 123});
   pros::delay(0);
   chassis.moveToRelativePoint(Chassis::fromPolar(0, -4), 10, {.forwards = false, .minSpeed = 60}, true);
   intake.setState(Intake::states::SCORING);
   chassis.moveToRelativePoint(0, 6, 500, {.forwards = false, .minSpeed = 55}, false);
   //pros::delay(300);
  //intake.setState(Intake::states::OUTTAKE);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.5), 100, {.forwards = false, .minSpeed = 100}, true);
   pros::delay(100);
  intake.setState(Intake::states::SCORING);
   pros::delay(1000);
 
   matchload_unloader.retract();
   chassis.cancelMotion();

   //! Reset to the goal
   auto left_goal = chassis.getPose();
   chassis.setPose({119.75, 48 - 7.25 + 2.5, left_goal.theta});

    // ! Lineup for Middle
   intake.setState(Intake::states::STORING);
   chassis.moveToPose(102, 36.5, -70, 1200, {.lead = 0.9, .minSpeed = 57}, false);
   //intake.setState(Intake::states::STORING);

   //! Grab Middle
   chassis.turnToPoint(98, 47.25, 400, {.minSpeed = 45});
   chassis.moveToPoint(98, 47.25, 1000, {.maxSpeed = 125, .minSpeed = 55}, true);
   pros::delay(200);
   matchload_unloader.extend();
   pros::delay(450);
   matchload_unloader.retract();

   chassis.turnToPoint(46, 6.5, 500, {.minSpeed = 52});
   chassis.moveToPoint(50, 47, 1300, {.maxSpeed = 125, .minSpeed = 55}, true);
   pros::delay(700);
   matchload_unloader.extend();
   pros::delay(300);
   //matchload_unloader.retract();

   chassis.moveToPoint(50, 46.25, 160, {.forwards = false, .maxSpeed = 90, .minSpeed = 50}, false);

   //! line up to score
   
   //chassis.turnToPoint(58+6, 54.5, 260, {.forwards = false, .minSpeed = 45});
   chassis.turnToHeading(-122, 470, {.minSpeed = 50}, false);
   //chassis.moveToPoint(58+8, 53.5, 700, {.forwards = false,  .minSpeed = 55}, true);
   chassis.moveToPoint(66, 54.2, 700, {.forwards = false,  .minSpeed = 55}, true);

   pros::delay(550);
   chassis.turnToPoint(24+11, 23.25, 500, { .minSpeed = 50});
   chassis.moveToPoint(67, 55.2, 700, {.forwards = false,  .minSpeed = 55}, true);

   // ! Score Middle
   middle_lift.retract();
   intake.setIntakeMultiplier(1.0, 1.0, 0.5);
   intake.setState(Intake::states::SCORING);
   //intake.setMiddle(true);
   pros::delay(720);
   intake.setIntakeMultiplier(1.0, 1.0, 1.0);
   intake.setState(Intake::states::STORING);

   // ! move off middle

   matchload_unloader.extend();

   chassis.turnToPoint(24+9.5, 24, 500, { .minSpeed = 52});
   chassis.moveToPoint(24+13.95, 24, 1100, { .maxSpeed = 110, .minSpeed = 90}, false);
//   // * loader clear 
   //! Lineup to long goal
   chassis.turnToHeading(180.0, 470, {.minSpeed = 52}, false);
  
    chassis.moveToPoint(24+13.95, -100.0, 760, {.maxSpeed = 69.5, .minSpeed = 62}, false);
//   // ! Attempt to score
// //   auto after_loader2 = chassis.getPose();
// //   chassis.moveToPoint(after_loader2.x -1.2, after_loader2.y + 36.0, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 90});
   middle_lift.extend();
   chassis.moveToRelativePoint(0.5, 48, 545, {.forwards = false, .minSpeed = 125}, false);
  
//   //middle_lift.extend();
 // chassis.moveToRelativePoint(0, 15, 200, {.forwards = false, .minSpeed = 56}, true);

  //! Scoring Long Goal
  intake.setState(Intake::states::SCORING);
   pros::delay(100);
    chassis.moveToRelativePoint(0, 6, 200, {.forwards = false, .minSpeed = 60}, true);
   intake.setState(Intake::states::SCORING);
   pros::delay(300);
   matchload_unloader.retract();
//  //intake.setState(Intake::states::OUTTAKE);
   pros::delay(100);
  intake.setState(Intake::states::SCORING);
   pros::delay(1600);



}

void leftSide(bool is_red_team)
{
  middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({48 + 7.25 + 1., 24, -90}, false);
  
  // ! De-score loader
  matchload_unloader.extend();
  pros::delay(200);  // Wait for the loader to extend before moving
  chassis.moveToPoint(29.95 , 24.0, 1000, {.maxSpeed = 90, .minSpeed = 40}, false);
  chassis.turnToHeading(-180.0, 500, {}, false);
  chassis.moveToPoint(30.1, -100.0, 1200, {.maxSpeed = 74, .minSpeed = 40}, false);

  // ! Attempt to score
  auto after_loader = chassis.getPose();
  chassis.moveToPoint(after_loader.x  , after_loader.y + 36.0, 1000, {.forwards = false, .maxSpeed = 75, .minSpeed = 40});
  pros::delay(100);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.23), 600, {.forwards = false, .minSpeed = 50}, true);
  intake.setState(Intake::states::SCORING);
  pros::delay(300);
 intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
 intake.setState(Intake::states::SCORING);
  pros::delay(1000);
 
  matchload_unloader.retract();
  chassis.cancelMotion();

  //! Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24+.5,48 - 7.25 + 2.5, left_goal.theta});
  pros::delay(100);
  chassis.moveToPoint(24.5, 30.25, 1000, {.forwards = true, .maxSpeed = 82, .minSpeed = 40}, true);

  // ! Grab Middle
  intake.setState(Intake::states::STORING);

//   chassis.moveToPose(40, 40, 90, 2000, {.lead = 0.5, .minSpeed = 40}, false);
//   intake.setState(Intake::states::STORING);
  chassis.turnToHeading(46, 1000, {.minSpeed = 11}, false);
//   chassis.moveToRelativePoint(Chassis::fromPolar(14 * sqrt(2), 51.5), 800, {.maxSpeed = 70, .minSpeed = 35}, true);
   chassis.moveToPoint(49, 53, 1000, {.forwards = true, .maxSpeed = 82, .minSpeed = 40}, true);
   pros::delay(550);
   matchload_unloader.extend();
  
   pros::delay(1200);

  // ! Lineup for Middle
  //chassis.turnToHeading(26 + 179+1.5+10, 1000, {.minSpeed = 10}, false);
  //chassis.moveToRelativePoint(Chassis::fromPolar(-13.5, 210.5), 1200, {.forwards = false, .minSpeed = 37}, false);
  chassis.turnToHeading(-135, 500, {.minSpeed = 10}, false);
  chassis.moveToPoint(60, 64, 1000, {.forwards = false, .maxSpeed = 82, .minSpeed = 40}, false);
  matchload_unloader.retract();
  middle_lift.retract();

  //! Scoring Middle
  intake.setIntakeMultiplier(1.0, 1.0, 0.29);
  intake.setState(Intake::states::SCORING);
  //intake.setMiddle(true);
  pros::delay(1800);
  intake.setIntakeMultiplier(1.0, 1.0, 1.0);

  //! Move away from middle Goal
  chassis.moveToPoint(41, 48.5, 1000, {.forwards = true, .maxSpeed = 82, .minSpeed = 40}, false);
  
  //! Lineup for swipe
  chassis.turnToHeading(180, 1000, {.minSpeed = 10}, false);
  
  //! Swipe long goal
  goal_descore_right.retract();
  chassis.moveToRelativePoint(-1.1, 19, 1000, {.forwards = false, .maxSpeed = 65}, false);
  chassis.moveToRelativePoint(0, -1, 1000, {.forwards = true, .maxSpeed = 65}, false);

  
}

void rightSide(bool is_red_team)
{
  middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({87.75, 24, 90}, false);
  
  //! De-score loader
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
   pros::delay(1300);
 
   matchload_unloader.retract();
   chassis.cancelMotion();

   //! Reset to the goal
   auto left_goal = chassis.getPose();
   chassis.setPose({119.75, 48 - 7.25 + 2.5, left_goal.theta});

    //! Grab Middle
   intake.setState(Intake::states::STORING);
   chassis.moveToPose(102, 36.5, -70, 1200, {.lead = 0.9, .minSpeed = 42}, false);
   //intake.setState(Intake::states::STORING);
   chassis.turnToPoint(98, 47.25, 400, {.minSpeed = 45});
   chassis.moveToPoint(96.25, 47.25, 1000, {.maxSpeed = 125, .minSpeed = 55}, true);
   pros::delay(200);

   //! grabbing blocks
   matchload_unloader.extend();
   pros::delay(390);
   matchload_unloader.retract();
   
   chassis.turnToHeading(-50, 640, {.maxSpeed = 60, .minSpeed = 47}, false);
   pros::delay(125);
   chassis.moveToRelativePoint(-20, 21.5, 1000, {.forwards = true, .maxSpeed = 60}, false);
   chassis.cancelMotion();
   intake.setIntakeMultiplier(.50, 1.0, 1.0);

   //! Scoring
   intake.setState(Intake::states::OUTTAKE);
   pros::delay(2500);
   intake.setIntakeMultiplier(1.0, 1.0, 1.0);

   //! Lining up to goal to swipe
   chassis.moveToRelativePoint(21.72, -20, 1000, {.forwards = false}, false);
   chassis.turnToHeading(0, 640, {.maxSpeed = 60, .minSpeed = 47}, false);
   goal_descore_right.retract();
   chassis.moveToRelativePoint(0, 22, 1000, {.forwards = true, .maxSpeed = 55}, false);
 
  
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
//! this is our move one inch!
{
  chassis.setPose({50, 12, -90});
   chassis.moveToPoint(46, 12, 2000, {.forwards = true, .minSpeed = 7}, false);

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
  chassis.moveToPoint(24.5-0.5-5 +4.5, -1000.0, 900, {.maxSpeed = 62, .minSpeed = 30}, false);
  chassis.turnToHeading(-187, 500, {}, false);
  chassis.moveToPoint(24.5-0.5-5 +5, -1000.0, 500, {.maxSpeed = 80, .minSpeed = 35}, false);
  chassis.turnToHeading(-180, 500, {}, false);
  chassis.moveToPoint(24.5-0.5-5 +5, -1000.0, 800, {.maxSpeed = 75, .minSpeed = 33}, false);

  chassis.moveToRelativePoint(0, 20-10, 1000, {.forwards = false}, false);
  intake.setState(Intake::states::STORING);
  matchload_unloader.retract();
  chassis.turnToHeading(-100+5, 600, {}, false);
  chassis.moveToRelativePoint(-17.5, 0, 710, {}, false);
  

  // ! RESET TO LEFT GOAL 1 using mcl
  mcl_localization.reset_particles({12, 34-10}, 8.0);
  pros::delay(650);

  auto left_goal_1 = chassis.getPose();
  chassis.setPose(
      mcl_localization.get_point_estimate().x,
      mcl_localization.get_point_estimate().y,
      left_goal_1.theta);

   // Go to loader 2
chassis.turnToHeading(-8.5, 600, {}, false);

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
 chassis.moveToPoint(7.35-3.25, 64, 2000, {.minSpeed = 10}, false);
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
    chassis.turnToPoint(9.1-2.5, 93, 700, {.minSpeed = 10}, false);
     chassis.moveToPoint(9.1-2.85, 93, 1300, {.minSpeed = 10}, false);
  chassis.turnToPoint(19.3, 113-8.5, 800, {.minSpeed = 30}, false);
  chassis.moveToPoint(19.2, 113-8.5, 900, {}, false);

    //* Score Loader 1
     chassis.turnToHeading(-0.2, 600, {}, false);

  middle_lift.extend();
  //chassis.moveToPoint(34, -50, 1000, {.forwards = false, .maxSpeed = 61.5, .minSpeed = 22}, false);
  //chassis.moveToRelativePoint(0, -5, 1000, {.forwards = false, .maxSpeed = 72}, true);
  chassis.moveToPoint(18.5, 95, 1000, {.forwards = false, .maxSpeed = 61.5, .minSpeed = 22}, true);
  chassis.moveToPoint(18.5, 87.5, 1000, {.forwards = false, .maxSpeed = 63, .minSpeed = 30}, true);
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
  chassis.moveToPoint(18, 1000, 1000, {.maxSpeed = 61.5, .minSpeed = 22}, true);
  pros::delay(200);
  middle_lift.retract();
  pros::delay(1500);

  chassis.moveToPoint(18.3, 1000, 500, {.maxSpeed = 60, .minSpeed = 20}, false);
  chassis.turnToHeading(11, 500, {.minSpeed = 20});
  chassis.moveToPoint(18.3, 1000, 500, {.maxSpeed = 70, .minSpeed = 30}, false);
  chassis.turnToHeading(0, 500, {.minSpeed = 20});
  chassis.moveToPoint(18.45, 500, 500, {.maxSpeed = 70, .minSpeed = 35}, false);
  pros::delay(1000);
   // SCORE GOAL 2
//*scoring second loader
 
 middle_lift.extend();
 chassis.moveToPoint(18.525, 89, 700, {.forwards = false, .maxSpeed = 70, .minSpeed = 30,}, false);

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
  chassis.moveToPoint(110.5, 121, 2500, {.maxSpeed = 77, .minSpeed = 35}, false);
  

  //* emptying loader 3
  chassis.turnToHeading(0, 500, {.minSpeed = 10});
  matchload_unloader.extend();
  pros::delay(200);
  
  chassis.moveToPoint(111.2, 1200, 900, {.maxSpeed = 60, .minSpeed = 35}, false);
  pros::delay(600);
  chassis.moveToPoint(111.35, 1200, 900, {.maxSpeed = 80, .minSpeed = 35}, false);
  pros::delay(600);
  chassis.moveToPoint(111.35, 1200, 900, {.maxSpeed = 70, .minSpeed = 35}, false);
  pros::delay(700);

  //* moving off loader
  chassis.moveToRelativePoint(0, -15, 1000, {.forwards = false, .minSpeed = 10}, false);
  matchload_unloader.retract();
  chassis.turnToHeading(90, 700, {.minSpeed = 10});
  chassis.moveToRelativePoint(33, 0, 1000, {.minSpeed = 10}, false);
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
  
 
  chassis.moveToRelativePoint(13.75, -110, 3000, {.forwards = true,.maxSpeed = 81, .minSpeed = 10}, false);

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

  chassis.moveToPoint(117.75, 65, 1000, {.forwards = false, .maxSpeed = 70, .minSpeed = 35}, false);
  intake.setState(Intake::states::SCORING);
  pros::delay(600);
  intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
  intake.setState(Intake::states::SCORING);
  pros::delay(1300);

  matchload_unloader.extend();
  intake.setState(Intake::states::STORING);

  //*grabbing 4th loader
  chassis.moveToPoint(124.7, -100, 1000, {.forwards = true, .maxSpeed = 60.5, .minSpeed = 32}, false);
  pros::delay(500);
  chassis.moveToPoint(123.7, -100, 1000, {.forwards = true, .maxSpeed = 75, .minSpeed = 35}, false);
  pros::delay(500);
  chassis.moveToPoint(123.5, -100, 1000, {.forwards = true, .maxSpeed = 72, .minSpeed = 35}, false);
  pros::delay(600);

  //* scoring 4th loader
  chassis.moveToRelativePoint(-2, 66, 1000, {.forwards = false, .maxSpeed = 65}, false);
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
  chassis.moveToPoint(102, -30, 700, {.forwards = true, .maxSpeed = 61.5, .minSpeed = 35}, false);
  chassis.moveToPoint(102, -50, 700, {.forwards = true, .maxSpeed = 61.5, .minSpeed = 35}, false);
  chassis.turnToHeading(-130, 400, {.minSpeed = 10});

  //* clearing park
  intake.setState(Intake::states::STORING);
  chassis.moveToRelativePoint(-66, -20.5, 1000, {.forwards = true, .maxSpeed = 75}, true);
  pros::delay(450);
  matchload_unloader.extend();
  pros::delay(350);
  chassis.moveToRelativePoint(-21, -5.85, 1100, {.forwards = true, .minSpeed = 35}, false);
  matchload_unloader.retract();
  chassis.moveToRelativePoint(1, 0, 1000, {.forwards = false, .minSpeed = 36}, false);
  


}


void VistaSkills(bool is_red_team)
{
middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({87.75, 24, 90}, false);
  
  // ? De-score loader
  matchload_unloader.extend();
  pros::delay(50);  // Wait for the loader to extend before moving used to be 200
  chassis.moveToPoint(115-1, 24.0, 1110, {.maxSpeed = 125, .minSpeed = 78}, false);
  chassis.turnToHeading(180.0, 640, {.maxSpeed = 60, .minSpeed = 47}, false);
    chassis.moveToPoint(115-1, -100.0, 3225, {.maxSpeed = 60, .minSpeed = 52}, false); //1225, {

   // ! Attempt to score
   auto after_loader = chassis.getPose();
   chassis.moveToPoint(after_loader.x +0.25, after_loader.y + 36.0, 2000, {.forwards = false, .maxSpeed = 125, .minSpeed = 45});
   pros::delay(0);
   chassis.moveToRelativePoint(Chassis::fromPolar(0, -4), 10, {.forwards = false, .minSpeed = 40}, true);
   //intake.setState(Intake::states::SCORING);
   pros::delay(300);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.5), 100, {.forwards = false, .minSpeed = 100}, true);
   pros::delay(100);
   //intake.setState(Intake::states::SCORING);
   //pros::delay(1100);
 
   matchload_unloader.retract();
   chassis.cancelMotion();

   // * Reset to the goal
   auto left_goal = chassis.getPose();
   chassis.setPose({119.75, 48 - 7.25 + 2.5, left_goal.theta});

    // ? Grab Middle
   intake.setState(Intake::states::STORING);
   chassis.moveToPose(102, 36.5, -70, 1200, {.lead = 0.9, .minSpeed = 42}, false);
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
   

   chassis.moveToPoint(49, 46.25, 160, {.forwards = false, .maxSpeed = 90, .minSpeed = 50}, false);

   // * line up to score
   //chassis.turnToPoint(58+4.7, 58, 260, {.forwards = false, .minSpeed = 45});
   //chassis.moveToPoint(58+4.7, 57.5, 700, {.forwards = false,  .minSpeed = 55}, false);


   // ! Score Middle
   //middle_lift.retract();
   //intake.setIntakeMultiplier(1.0, 1.0, 0.29);
   //intake.setState(Intake::states::SCORING);
   //intake.setMiddle(true);
   //pros::delay(700);
   //intake.setIntakeMultiplier(1.0, 1.0, 1.0);
   //intake.setState(Intake::states::STORING);

   // ? go to other loader 

   matchload_unloader.extend();

   chassis.turnToPoint(24+9.5, 24, 1000, { .minSpeed = 35});
   chassis.moveToPoint(24+9.5, 24, 2100, { .maxSpeed = 90, .minSpeed = 40}, false);

//   // * loader clear 


   chassis.turnToHeading(180.0, 600, {.minSpeed = 35}, false);

  // ! score first time in second goal
 
   middle_lift.extend();
   chassis.moveToRelativePoint(0.2, 28, 2000, {.forwards = false, .minSpeed = 45}, false);

   //auto after_loader = chassis.getPose();
  chassis.moveToPoint(after_loader.x , after_loader.y + 36.0, 1600, {.forwards = false, .maxSpeed = 75, .minSpeed = 30});
  pros::delay(600);

   intake.setState(Intake::states::SCORING);
   pros::delay(3000);
   intake.setState(Intake::states::STORING);

  // ? get match lader
    chassis.moveToPoint(24+9.75, -150.0, 2500, {.maxSpeed = 68, .minSpeed = 59}, false); //760, {

//   // ! Attempt to score
// //   auto after_loader2 = chassis.getPose();
// //   chassis.moveToPoint(after_loader2.x -1.2, after_loader2.y + 36.0, 1000, {.forwards = false, .maxSpeed = 127, .minSpeed = 90});
   middle_lift.extend();
   chassis.moveToRelativePoint(0.2, 48, 2000, {.forwards = false, .minSpeed = 45}, false);
  
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
   pros::delay(3600);
}

 void LeftSide7Block(bool is_red_team)
{

  middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({54+1, 15.25, 0}, false);
  
  // ! snatch 3 blocks

  chassis.moveToPoint(48 , 42, 1300, {.maxSpeed = 110, .minSpeed = 55}, true);
  pros::delay(500);
  matchload_unloader.extend();
  pros::delay(800);

  //! get loader
  chassis.turnToPoint(28+16, 24, 1000,{.maxSpeed =90, .minSpeed = 30}, false);
  chassis.moveToPoint(28.5+2, 24.0, 1300, {.maxSpeed = 110, .minSpeed = 50}, false);
  chassis.turnToHeading(-180.0, 500, {}, false);
  chassis.moveToPoint(28.5+2, -100.0, 1200, {.maxSpeed = 74, .minSpeed = 40}, false);

  //! score long goal
   auto after_loader = chassis.getPose();
  chassis.moveToPoint(after_loader.x  , after_loader.y + 36.0, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 45});
  pros::delay(100);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.23), 600, {.forwards = false, .minSpeed = 50}, true);
  intake.setState(Intake::states::SCORING);
  pros::delay(300);
 intake.setState(Intake::states::OUTTAKE);
  pros::delay(170);
 intake.setState(Intake::states::SCORING);
  pros::delay(1800);
 
  matchload_unloader.retract();
  chassis.cancelMotion();

  // ! Reset to the goal
  auto left_goal = chassis.getPose();
  chassis.setPose({24+.5,48 - 7.25 + 2.5, left_goal.theta});
  pros::delay(100);

  intake.setState(Intake::states::STORING);
  //! swipe  

  goal_descore_right.retract();
  chassis.moveToPoint(34.25, 38.5, 1000, {.maxSpeed = 110, .minSpeed = 50}, false);
  chassis.turnToHeading(-180.0, 500, {}, false);

 chassis.moveToPoint(34.25, 60, 3000,{.forwards = false, .maxSpeed = 42, .minSpeed = 10}, false);


 chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
 pros::delay(500);
 
 chassis.moveToPoint(34.25, 61, 15000,{.forwards = false, .maxSpeed = 10, .minSpeed = 1}, false);
 //chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}





void RightSide7Block(bool is_red_team)
{
  middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({89, 15.25, 0}, false);
  
  // ! snatch 3 blocks

  chassis.moveToPoint(94.5 , 43, 1300, {.maxSpeed = 110, .minSpeed = 55}, true);
  pros::delay(470);
  matchload_unloader.extend();
  pros::delay(730);

  //! get loader
  chassis.turnToPoint(96, 26, 1000,{.maxSpeed =90, .minSpeed = 30}, false);
  chassis.moveToPoint(117, 28.0, 1300, {.maxSpeed = 110, .minSpeed = 50}, false);
  chassis.turnToHeading(-180.0, 500, {}, false);
  chassis.moveToPoint(117, -100.0, 1200, {.maxSpeed = 74, .minSpeed = 40}, false);

  //! score long goal
   auto after_loader = chassis.getPose();
  chassis.moveToPoint(after_loader.x +0.75 , after_loader.y + 36.0, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 45});
  pros::delay(100);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.23), 600, {.forwards = false, .minSpeed = 50}, true);
  intake.setState(Intake::states::SCORING);
  pros::delay(300);
 intake.setState(Intake::states::OUTTAKE);
  pros::delay(100);
 intake.setState(Intake::states::SCORING);
  pros::delay(1500);
 
  matchload_unloader.retract();
  chassis.cancelMotion();

  // ! Reset to the goal
  auto right_goal = chassis.getPose();
  chassis.setPose({119.5, 48 - 7.25 + 2.5, right_goal.theta});
  pros::delay(100);

  intake.setState(Intake::states::STORING);
  //! swipe  

  goal_descore_right.retract();
  chassis.moveToPoint(109.57, 38.5, 1000, {.maxSpeed = 110, .minSpeed = 50}, false);
  chassis.turnToHeading(0.0, 500, {}, false);

  chassis.moveToPoint(110, 60, 3000,{.maxSpeed = 40, .minSpeed = 10}, false);


 chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
 pros::delay(500);
 chassis.moveToPoint(110, 61, 15000,{.maxSpeed = 10, .minSpeed = 1}, false);
}





void RightSideDoubleMiddle(bool is_red_team)
{

  middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({87.75, 24, 90}, false);
  
  // ! De-score loader
  matchload_unloader.extend();
  pros::delay(50);  // Wait for the loader to extend before moving used to be 200
  chassis.moveToPoint(115.25, 24.0, 1100, {.maxSpeed = 120, .minSpeed = 65}, false);
  chassis.turnToHeading(180.0, 700, {.maxSpeed = 80, .minSpeed = 50}, false);
    chassis.moveToPoint(115.25, -100.0, 1000, {.maxSpeed = 65, .minSpeed = 30}, false);

   // ! Attempt to score
   auto after_loader = chassis.getPose();
   chassis.moveToPoint(after_loader.x +0.25, after_loader.y + 36.0, 700, {.forwards = false, .maxSpeed = 125, .minSpeed = 100});
   pros::delay(0);
   chassis.moveToRelativePoint(Chassis::fromPolar(0, -4), 100, {.forwards = false, .minSpeed = 40}, true);
   // intake.setState(Intake::states::SCORING);
   // pros::delay(300);
  //intake.setState(Intake::states::OUTTAKE);
   chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.5), 100, {.forwards = false, .minSpeed = 100}, true);
   pros::delay(100);
//   intake.setState(Intake::states::SCORING);
//    pros::delay(1300);
 
   matchload_unloader.retract();
   chassis.cancelMotion();

   // ! Reset to the goal
   auto left_goal = chassis.getPose();
   chassis.setPose({119.75, 48 - 7.25 + 2.5, left_goal.theta});

    // ! Grab Middle
   intake.setState(Intake::states::STORING);
   chassis.moveToPose(102, 36.5, -70, 1200, {.lead = 0.9, .minSpeed = 42}, false);
   //intake.setState(Intake::states::STORING);
   chassis.turnToPoint(98, 47.25, 300, {.minSpeed = 55});
   chassis.moveToPoint(96.25, 47.25, 900, {.maxSpeed = 125, .minSpeed = 65}, true);
   pros::delay(200);
   matchload_unloader.extend();
   pros::delay(450);
   matchload_unloader.retract();

   //! grab under long goal blocks
   chassis.turnToPoint(112, 63.2, 900, { .maxSpeed = 110, .minSpeed = 65}, false);
   matchload_unloader.retract();
   chassis.moveToPoint(112, 63.2, 900, { .maxSpeed = 110, .minSpeed = 65}, true);
   pros::delay(600);
   matchload_unloader.extend();
  
   chassis.turnToHeading(78, 300, { .maxSpeed = 100, .minSpeed = 50});
   chassis.turnToHeading(86, 300, { .maxSpeed = 100, .minSpeed = 50});
   chassis.turnToHeading(48, 300, { .maxSpeed = 100, .minSpeed = 50});
   chassis.moveToPoint(113, 63.2, 300, { .maxSpeed = 50, .minSpeed = 20}, true);
   chassis.turnToHeading(73, 300, { .maxSpeed = 100, .minSpeed = 50});
   pros::delay(300);
   chassis.moveToPoint(100, 50, 1000, {.forwards= false, .maxSpeed = 120, .minSpeed = 70}, true);
   matchload_unloader.retract();

   //! score bottom middle
   chassis.turnToPoint(90, 60, 300, {.minSpeed = 55});
   chassis.moveToPoint(89, 61, 900, {.maxSpeed = 125, .minSpeed = 65}, true);

   pros::delay(150);
   intake.setState(Intake::states::OUTTAKE);
   pros::delay(1100);
   intake.setState(Intake::states::STORING);

   //! moving to middle
   chassis.moveToPoint(90, 45, 800, {.forwards = false, .maxSpeed = 125, .minSpeed = 75}, false);
   chassis.moveToPoint(50, 50, 1200, {.forwards = true, .maxSpeed = 125, .minSpeed = 80}, false);
  
   chassis.turnToHeading(-135, 300, { .maxSpeed = 100, .minSpeed = 50});
  
   chassis.moveToPoint(69, 59, 900, {.forwards = false, .maxSpeed = 125, .minSpeed = 65}, false);

   middle_lift.retract();
   intake.setIntakeMultiplier(1.0, 1.0, 0.5);
   intake.setState(Intake::states::SCORING);
   
}





void RightSide9Block(bool is_red_team)
{
  middle_lift.extend();
  intake.setState(Intake::states::STORING);

  chassis.setPose({89, 15.25, 0}, false);
  
  // ! snatch 3 blocks

  chassis.moveToPoint(94.5 , 43, 1300, {.maxSpeed = 110, .minSpeed = 55}, true);
  pros::delay(450);
  matchload_unloader.extend();
  pros::delay(800);

  chassis.turnToPoint(112, 63.2, 1300, { .maxSpeed = 110, .minSpeed = 55}, false);
  matchload_unloader.retract();
  chassis.moveToPoint(112, 63.2, 1300, { .maxSpeed = 110, .minSpeed = 55}, true);
  pros::delay(300);
  matchload_unloader.extend();
  
  chassis.turnToHeading(73, 500);
  chassis.turnToHeading(81, 500);
  chassis.turnToHeading(50, 500);
  chassis.moveToPoint(113, 64.2, 1300, { .maxSpeed = 110, .minSpeed = 55}, true);
  chassis.turnToHeading(73, 500);
  pros::delay(500);
  chassis.moveToPoint(98 , 50, 1300, {.forwards= false, .maxSpeed = 110, .minSpeed = 55}, true);


   //! get loader
   chassis.turnToPoint(96, 26, 1000,{.maxSpeed =90, .minSpeed = 30}, false);
  chassis.moveToPoint(117.5, 32.0, 1300, {.maxSpeed = 110, .minSpeed = 50}, false);
  chassis.turnToHeading(-180.0, 500, {}, false);
 chassis.moveToPoint(117.5, -100.0, 1200, {.maxSpeed = 74, .minSpeed = 40}, false);

   //! score long goal
  auto after_loader = chassis.getPose();
 chassis.moveToPoint(after_loader.x +0.2, after_loader.y + 36.0, 1000, {.forwards = false, .maxSpeed = 100, .minSpeed = 45});
 pros::delay(100);
  chassis.moveToRelativePoint(Chassis::fromPolar(0, -0.23), 600, {.forwards = false, .minSpeed = 50}, true);
  intake.setState(Intake::states::SCORING);
  pros::delay(300);
 intake.setState(Intake::states::OUTTAKE);
  pros::delay(200);
 intake.setState(Intake::states::SCORING);
  pros::delay(2000);
 
  matchload_unloader.retract();
  chassis.cancelMotion();
  middle_lift.retract();

//   // ! Reset to the goal
//   auto right_goal = chassis.getPose();
//   chassis.setPose({119.5, 48 - 7.25 + 2.5, right_goal.theta});
//   pros::delay(100);

//   intake.setState(Intake::states::STORING);
//   //! swipe  

//   goal_descore_right.retract();
//   chassis.moveToPoint(110, 38.5, 1000, {.maxSpeed = 110, .minSpeed = 50}, false);
//   chassis.turnToHeading(0.0, 500, {}, false);

//   chassis.moveToPoint(110, 60, 3000,{.maxSpeed = 40, .minSpeed = 10}, false);


//  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
//  pros::delay(500);
//  chassis.moveToPoint(110, 61, 15000,{.maxSpeed = 10, .minSpeed = 1}, false);
}
