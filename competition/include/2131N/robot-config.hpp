#pragma once

#include "2131N/systems/chassis.hpp"
#include "2131N/systems/intake.hpp"
#include "2131N/systems/mcl/time_of_flight.hpp"
#include "2131N/ui/screen.hpp"
#include "pros/rotation.hpp"
#include "systems/mcl/mcl.hpp"
#include "systems/lift.hpp"


extern pros::Controller primary;

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::MotorGroup lift;
extern pros::Motor Mclaw;
extern pros::Rotation MclawRotation;

extern pros::adi::Pneumatics matchload_unloader;

extern pros::adi::Pneumatics middle_lift;
extern pros::adi::Pneumatics goal_descore_right;
extern pros::adi::Pneumatics middle_descore;
//extern pros::adi::Pneumatics first_stage_lift;
extern pros::adi::Pneumatics storage_block;
extern pros::adi::Pneumatics legoclaww;
extern pros::adi::Pneumatics flipclaw;

extern Lift DR4B;
extern Twister Wrist;

//pros::MotorGroup* bottom_stage_;

extern Chassis chassis;

extern DistanceSensor left_distance;
extern DistanceSensor right_distance;
extern DistanceSensor back_distance;
extern DistanceSensor front_distance;

extern Intake intake;
extern Screen screen;

extern Mcl<800> mcl_localization;
