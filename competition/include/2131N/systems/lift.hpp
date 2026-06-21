#pragma once

#include "2131N/robot-config.hpp"
#include "pros/adi.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

class Lift 
{
    private:
        pros::MotorGroup& m_liftMotors;

    public:
        Lift(pros::MotorGroup& LiftMotors) : m_liftMotors(LiftMotors)
        {
            m_liftMotors.set_zero_position_all(0);
            m_liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

        }

        enum class liftStates {
            STAGE_1 = 0,
            STAGE_2 = 200,
            STAGE_3 = 400, 
        };

        void setState(liftStates state)
        {
            m_liftMotors.move_absolute(int(state), 1200);
            m_liftMotors.brake();
        }





};

class Twister
{
    public:
         enum class twisterStates {
            IN = -50,
            OUT = 500,
        };
    private:
        pros::Rotation& m_sensor;
        pros::Motor& m_motor;
        twisterStates m_currentState;
    public:
        Twister(pros::Motor& Motor, pros::Rotation& rotation) : m_sensor(rotation), m_motor(Motor) {

            pros::Task task([=, this]{
                const float kP = 0.0;
                while(true)
                {
                    float error = int(m_currentState) - m_sensor.get_position();
                    m_motor.move_voltage(error*kP);
                    pros::delay(20);
                }
            });
        }

       

        void setState(twisterStates state) {
            m_currentState = state;



        }


};
