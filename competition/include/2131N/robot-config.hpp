#pragma once

#include "2131N/systems/intake.hpp"
#include "2131N/ui/screen.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"

extern pros::Controller primary;
extern pros::adi::Pneumatics matchload_unloader;

extern pros::Distance right_distance;
extern pros::Distance left_distance;
extern pros::Distance back_distance;

extern lemlib::Chassis chassis;

extern Intake intake;
extern Screen screen;
