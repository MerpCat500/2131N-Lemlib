#include "robot-config.hpp"

#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motor_group.hpp"

pros::MotorGroup leftMotors({-10, -9, -8}, pros::MotorGear::blue, pros::MotorUnits::deg);
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGear::blue, pros::MotorUnits::deg);

pros::Motor hopper(4, pros::MotorGear::blue, pros::MotorUnits::deg);
pros::Motor btmStage(5, pros::MotorGear::blue, pros::MotorUnits::deg);
pros::Motor topStage(6, pros::MotorGear::blue, pros::MotorUnits::deg);

pros::Imu inertial(21);

pros::Controller primary(pros::E_CONTROLLER_MASTER);

lemlib::OdomSensors sensors{
    nullptr,
    nullptr,  //
    nullptr,
    nullptr,  //
    &inertial};

lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 13.25, 2.75, 450.0, 8.0);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    10,   // proportional gain (kP)
    0,    // integral gain (kI)
    3,    // derivative gain (kD)
    3,    // anti windup
    1,    // small error range, in inches
    100,  // small error range timeout, in milliseconds
    3,    // large error range, in inches
    500,  // large error range timeout, in milliseconds
    20    // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    2,    // proportional gain (kP)
    0,    // integral gain (kI)
    10,   // derivative gain (kD)
    3,    // anti windup
    1,    // small error range, in degrees
    100,  // small error range timeout, in milliseconds
    3,    // large error range, in degrees
    500,  // large error range timeout, in milliseconds
    0     // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
