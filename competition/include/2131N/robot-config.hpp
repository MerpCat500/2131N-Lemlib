#pragma once

#include "2131N/systems/chassis.hpp"
#include "2131N/systems/intake.hpp"
#include "2131N/systems/mcl/time_of_flight.hpp"
#include "2131N/ui/screen.hpp"
#include "systems/mcl/mcl.hpp"

extern pros::Controller primary;

extern pros::adi::Pneumatics matchload_unloader;

extern pros::adi::Pneumatics goal_descore_left;
extern pros::adi::Pneumatics goal_descore_right;
extern pros::adi::Pneumatics middleGoalFlap;
extern pros::adi::Pneumatics first_stage_lift;

extern Chassis chassis;

extern DistanceSensor left_distance;
extern DistanceSensor right_distance;
extern DistanceSensor back_distance;
extern DistanceSensor front_distance;

extern Intake intake;
extern Screen screen;

extern Mcl<800> mcl_localization;
