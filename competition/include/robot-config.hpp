#pragma once

#include "lemlib/chassis/chassis.hpp"

extern lemlib::Chassis chassis;
extern pros::Controller primary;

extern pros::Motor hopper;
extern pros::Motor btmStage;
extern pros::Motor topStage;

#define HOPPER_BTN_IN pros::E_CONTROLLER_DIGITAL_A
#define HOPPER_BTN_OUT pros::E_CONTROLLER_DIGITAL_B

#define BTM_STAGE_BTN_IN pros::E_CONTROLLER_DIGITAL_L2
#define BTM_STAGE_BTN_OUT pros::E_CONTROLLER_DIGITAL_L1

#define TOP_STAGE_BTN_IN pros::E_CONTROLLER_DIGITAL_R2
#define TOP_STAGE_BTN_OUT pros::E_CONTROLLER_DIGITAL_R1