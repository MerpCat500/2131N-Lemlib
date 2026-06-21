/**
 * @file intake.hpp
 * @author Andrew Hilton (2131N)
 * @brief Declaration of the Intake Class
 * @version 0.1
 * @date 2025-07-25
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include "2131N/utils/change_detector.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"


class Intake
{
 public:  // State System
 Intake(pros::MotorGroup* motor_group,
           float speed,
           pros::Controller* controller,
           pros::controller_digital_e_t btn_in,
           pros::controller_digital_e_t btn_out,
           pros::controller_digital_e_t btn_r1,
           pros::controller_digital_e_t btn_r2);

 private:
 pros::MotorGroup* motor_group_;
    float speed_;
    pros::Controller* controller_;
    pros::controller_digital_e_t btn_in_, btn_out_, btn_r1_, btn_r2_;

  pros::MotorGroup* bottom_stage_;  // Pointer to the bottom stage motor
  pros::Motor* middle_stage_;  // Pointer to the storage motor
  pros::Motor* top_stage_;     // Pointer to the top stage motor

  pros::adi::Pneumatics* middle_stage_gate_;  // Middle stage gate
  // pros::adi::Pneumatics* first_stage_lift;

  pros::Distance* bottom_detector_;  // Pointer to the bottom stage detector
  float detection_range_;            // Anything less than this number will be counted as detected
  bool ball_detected_ = false;       // Is the detector reading a ball
  ChangeDetector<bool> ball_detector;

  pros::Controller* primary_;  // Controller (for tele-op)

  pros::controller_digital_e_t
      intake_button_;  // Intake Button (Spins Top and Bottom Stage to cycle ball up)
  pros::controller_digital_e_t
      outtake_button_;  // Outtake Button (Spins Top and Bottom Stage to cycle ball down)

  pros::controller_digital_e_t score_top_button_;  // Store Button (Spins Storage in to store balls)
  pros::controller_digital_e_t
      score_middle_button;  // Unstore Button (Spins Storage out to remove stored balls)

  pros::Task update_thread_;

  bool score_mode_ = false;
  bool score_middle_ = false;
  bool anti_jam_ = false;
  size_t jam_loop_ = 0;

  double intake_multipliers[3] = {1.0, 1.0, 1.0};

 public:
  enum class states
  {
    STOPPED,
    OUTTAKE,
    OUTTAKEMIDDLE,
    STORING,
    SCORING,
    SCORE_MIDDLE,
    STORE_TOP
  } state;

 public:
  Intake(
      pros::MotorGroup* bottom_stage,
      pros::Motor* middle_stage,
      pros::Motor* top_stage,
      pros::Distance* bottom_detector,
      pros::adi::Pneumatics* middle_gate,
      float detection_range,
      pros::Controller* primary,
      pros::controller_digital_e_t intake_button,
      pros::controller_digital_e_t outtake_button,
      pros::controller_digital_e_t score_top_button,
      pros::controller_digital_e_t score_middle_button)
      : bottom_stage_(bottom_stage),
        middle_stage_(middle_stage),
        top_stage_(top_stage),
        middle_stage_gate_(middle_gate),
        bottom_detector_(bottom_detector),
        detection_range_(detection_range),
        primary_(primary),
        intake_button_(intake_button),
        outtake_button_(outtake_button),
        score_top_button_(score_top_button),
        score_middle_button(score_middle_button),
        update_thread_(
            [this]() {
              while (true)
              {
                this->update();
                pros::delay(10);
              }
            },
            "Intake Update")
  {
  }






void teleOp()
  {
    if (primary_->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
    {
       setState(states::OUTTAKEMIDDLE); 
    }
    
    else if (primary_->get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { 
      //topIntakeSpeed = 12000;
      setState(states::OUTTAKE); 
    }


    //     else if(primary.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_B))
    // {
     
    //  bottom_stage_->move_relative(10, 50);
     
    // }

    
    // else if (primary_->get_digital_new_press(score_top_button_))
    //  {
    //    setState(states::STOPPED);
    //  }

    //   if (primary_->get_digital_new_press(score_top_button_))
    // {
    //   // if(scoreSpeedPressed){
    //   //   topIntakeSpeed = 4000;
    //   // }
    //   // else{
    //   //   topIntakeSpeed = 4000;
    //   // }
    //   score_mode_ = !score_mode_;
    //   if (primary_->get_digital(intake_button_))
    //   {
    //      if (score_mode_) { setState(states::SCORING); }
    //      else
    //     {
    //       setState(states::STORING);
    //       middle_stage_->brake();
    //     }
    //   }
    // }
  }


  void setState(states new_state) { state = new_state; }
  void setMiddle(bool v) { middle_stage_gate_->set_value(v); }
  void setIntakeMultiplier(double scale1, double scale2, double scale3) { this->intake_multipliers[0] = scale1; this->intake_multipliers[1] = scale2; this->intake_multipliers[2] = scale3; }
  void antiJam(bool anti_jam)
  {
    anti_jam_ = anti_jam;
    jam_loop_ = 0;
  }

 private:
  void update()
  {
  
      switch (state)
      {
      
        case states::OUTTAKE: 

          bottom_stage_->move_voltage(-12000 * intake_multipliers[0]);
          
          break;
        case states::OUTTAKEMIDDLE:

          bottom_stage_->move_voltage(12000 * intake_multipliers[0]);
         
          break;
        
       
    }
  }
};