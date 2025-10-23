#include "2131N/robot-config.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"

pros::MotorGroup left_motors({-2, -3, 4}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::deg);
pros::MotorGroup right_motors({5, 6, -7}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::deg);
pros::Imu inertial(21);

pros::Motor firstStage(10);
pros::Motor secondStage(9);
pros::Motor thirdStage(1);

pros::Distance btmStorageDetector(8);

pros::adi::Pneumatics middleGoalFlap('A', false);
pros::adi::Pneumatics matchload_unloader('B', false);

pros::Controller primary(pros::E_CONTROLLER_MASTER);

lemlib::OdomSensors sensors{
    nullptr,
    nullptr,  //
    nullptr,
    nullptr,  //
    &inertial};

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 10.75, 3.4, 400.0, 10.0);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    9.4117,  // proportional gain (kP)
    0,       // integral gain (kI)
    2.8235,  // derivative gain (kD)
    3,       // anti windup
    1,       // small error range, in inches
    100,     // small error range timeout, in milliseconds
    3,       // large error range, in inches
    500,     // large error range timeout, in milliseconds
    10       // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    1.8823,  // proportional gain (kP)
    0,       // integral gain (kI)
    9.4117,  // derivative gain (kD)
    3,       // anti windup
    1,       // small error range, in degrees
    100,     // small error range timeout, in milliseconds
    3,       // large error range, in degrees
    500,     // large error range timeout, in milliseconds
    0        // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

Intake intake(
    &firstStage,
    &secondStage,
    &thirdStage,
    &btmStorageDetector,
    &middleGoalFlap,
    85.0f,
    &primary,
    pros::E_CONTROLLER_DIGITAL_L2,
    pros::E_CONTROLLER_DIGITAL_L1,
    pros::E_CONTROLLER_DIGITAL_R1,
    pros::E_CONTROLLER_DIGITAL_R2);

Screen screen;
