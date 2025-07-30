#include "2131N/robot-config.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "pros/motor_group.hpp"

pros::MotorGroup left_motors({10, -9, -8}, pros::MotorGear::blue, pros::MotorUnits::deg);
pros::MotorGroup right_motors({-1, 2, 3}, pros::MotorGear::blue, pros::MotorUnits::deg);
pros::Imu inertial(21);

pros::Motor hopper(-11, pros::MotorGear::blue, pros::MotorUnits::deg);
pros::Motor btmStage(20, pros::MotorGear::blue, pros::MotorUnits::deg);
pros::Motor topStage(-12, pros::MotorGear::blue, pros::MotorUnits::deg);

pros::Distance right_distance(5);
pros::Distance left_distance(7);
pros::Distance back_distance(18);

pros::adi::Pneumatics matchload_unloader('H', false);
pros::adi::Pneumatics storage_unstore('A', false);

pros::Controller primary(pros::E_CONTROLLER_MASTER);

lemlib::OdomSensors sensors{
    nullptr,
    nullptr,  //
    nullptr,
    nullptr,  //
    &inertial};

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 10, 3.3825, 400.0, 8.0);

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
    10    // maximum acceleration (slew)
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

Intake intake(
    &topStage,
    &btmStage,
    &hopper,
    {},
    &primary,
    pros::E_CONTROLLER_DIGITAL_L2,
    pros::E_CONTROLLER_DIGITAL_L1,
    pros::E_CONTROLLER_DIGITAL_R2,
    pros::E_CONTROLLER_DIGITAL_R1,
    150.0f,
    20.0f);

Screen screen;
