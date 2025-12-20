#include "2131N/robot-config.hpp"

#include "lemlib/chassis/chassis.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "systems/chassis.hpp"

pros::MotorGroup left_motors({-7, 8, -9}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::deg);
pros::MotorGroup right_motors({4, -5, 6}, pros::v5::MotorGears::blue, pros::v5::MotorUnits::deg);
pros::Imu inertial(21);

pros::Motor firstStage(-3);
pros::Motor secondStage(-1);
pros::Motor thirdStage(2);

pros::Distance btmStorageDetector(18);

pros::adi::Pneumatics goal_descore_right('A', false);
pros::adi::Pneumatics middleGoalFlap('B', false);
pros::adi::Pneumatics matchload_unloader('C', false);
pros::adi::Pneumatics goal_descore_left('D', false);

pros::Controller primary(pros::E_CONTROLLER_MASTER);

lemlib::OdomSensors sensors{
    nullptr,
    nullptr,  //
    nullptr,
    nullptr,  //
    &inertial};

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, 10.75, 3.258333332, 400.0, 10.0);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    7.5,  // proportional gain (kP)
    0.2,  // integral gain (kI)
    9,    // derivative gain (kD)
    1,    // anti windup
    1,    // small error range, in inches
    200,  // small error range timeout, in milliseconds
    2,    // large error range, in inches
    400,  // large error range timeout, in milliseconds
    10    // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(
    2.2,   // proportional gain (kP)
    0.25,  // integral gain (kI)
    14,    // derivative gain (kD)
    5,     // anti windup
    2,     // small error range, in degrees
    200,   // small error range timeout, in milliseconds
    3,     // large error range, in degrees
    500,   // large error range timeout, in milliseconds
    0      // maximum acceleration (slew)
);

Chassis chassis({drivetrain, lateral_controller, angular_controller, sensors});

Intake intake(
    &firstStage,
    &secondStage,
    &thirdStage,
    &btmStorageDetector,
    &middleGoalFlap,
    110.0f,
    &primary,
    pros::E_CONTROLLER_DIGITAL_L2,
    pros::E_CONTROLLER_DIGITAL_L1,
    pros::E_CONTROLLER_DIGITAL_R1,
    pros::E_CONTROLLER_DIGITAL_R2);

Screen screen;

DistanceSensor left_distance({-1.0, 5.5}, -M_PI_2, 17);
DistanceSensor right_distance({-0.5, -5.5}, M_PI_2, 19);
DistanceSensor back_distance({-1.5, 6.5}, M_PI, 20);
DistanceSensor front_distance({-1.5, 6.5}, 0, 16);

Mcl<800> mcl_localization(
    &chassis,
    std::make_shared<Field>(Field(Point(1, 1), Point(143, 143))),
    std::vector<DistanceSensor*>{
        &left_distance,
        &back_distance,
        &right_distance,
    });