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
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"

class Intake
{
 public:  // State System
 private:
  pros::Motor* bottom_stage_;  // Pointer to the bottom stage motor
  pros::Motor* middle_stage_;  // Pointer to the storage motor
  pros::Motor* top_stage_;     // Pointer to the top stage motor

  pros::adi::Pneumatics* middle_stage_gate_;  // Middle stage gate

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

  // pros::controller_digital_e_t just_the_fucking_bottom_button_;

  pros::Task update_thread_;

  bool score_mode_ = false;
  bool score_middle_ = false;

 public:
  enum class states
  {
    STOPPED,
    // JUST_THE_FUCKING_BOTTOM,
    OUTTAKE,
    STORING,
    SCORING,
    SCORE_MIDDLE,
    STORE_TOP
  } state;

 public:
  Intake(
      pros::Motor* bottom_stage,
      pros::Motor* middle_stage,
      pros::Motor* top_stage,
      pros::Distance* bottom_detector,
      pros::adi::Pneumatics* middle_gate,
      float detection_range,
      pros::Controller* primary,
      pros::controller_digital_e_t intake_button,
      pros::controller_digital_e_t outtake_button,
      pros::controller_digital_e_t score_top_button,
      pros::controller_digital_e_t score_middle_button
      // pros::controller_digital_e_t just_the_fucking_bottom_button_
      )
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
    if (primary_->get_digital_new_press(intake_button_))
    {
      if (score_middle_) { setState(states::SCORE_MIDDLE); }
      else if (score_mode_) { setState(states::SCORING); }
      else { setState(states::STORING); }
    }
    else if (primary_->get_digital_new_press(outtake_button_)) { setState(states::OUTTAKE); }
    // else if (primary_->get_digital_new_press(just_the_fucking_bottom_button_))
    // {
    //   std::cout << "JUST THE FUCKING BOTTOM" << std::endl;
    //   setState(states::JUST_THE_FUCKING_BOTTOM);
    // }
    else if (
        primary_->get_digital_new_release(intake_button_) ||
        primary_->get_digital_new_release(outtake_button_)
        //|| primary_->get_digital_new_release(just_the_fucking_bottom_button_)
    )
    {
      setState(states::STOPPED);
    }

    if (primary_->get_digital_new_press(score_top_button_))
    {
      score_middle_ = false;
      middle_stage_gate_->set_value(score_middle_);

      score_mode_ = !score_mode_;
      if (primary_->get_digital(intake_button_))
      {
        if (score_middle_) { setState(states::SCORE_MIDDLE); }
        else if (score_mode_) { setState(states::SCORING); }
        else
        {
          setState(states::STORING);
          middle_stage_->brake();
        }
      }
    }
    else if (primary_->get_digital_new_press(score_middle_button))
    {
      score_middle_ = !score_middle_;

      middle_stage_gate_->set_value(score_middle_);

      if (score_middle_ == false) { score_mode_ = false; }
      if (primary_->get_digital(intake_button_))
      {
        if (score_middle_) { setState(states::SCORE_MIDDLE); }
        else if (score_mode_) { setState(states::SCORING); }
        else
        {
          setState(states::STORING);
          middle_stage_->brake();
        }
      }
    }
  }

  void setState(states new_state) { state = new_state; }
  void setMiddle(bool v) { middle_stage_gate_->set_value(v); }

 private:
  void update()
  {
    ball_detected_ = (bottom_detector_->get() < detection_range_);
    ball_detector.checkValue(ball_detected_);

    switch (state)
    {
      case states::STORING:

        bottom_stage_->move_voltage(12000);
        top_stage_->move_voltage(-1000);

        if (ball_detector.getValue())
        {
          middle_stage_->set_encoder_units_all(pros::MotorEncoderUnits::deg);
          middle_stage_->move_velocity(110);
          if (std::abs(middle_stage_->get_torque()) > 0.9)
          {
            middle_stage_->set_brake_mode_all(pros::MotorBrake::coast);
            middle_stage_->brake();
          }
        }
        else { middle_stage_->brake(); }

        break;
      case states::SCORE_MIDDLE:

        // TODO: Add reversing behavior
        bottom_stage_->move_voltage(12000);
        middle_stage_->move_voltage(8000);
        top_stage_->move_voltage(-6000);
        break;
      case states::OUTTAKE:

        bottom_stage_->move_voltage(-12000);
        middle_stage_->move_voltage(-12000);
        top_stage_->move_voltage(-12000);
        break;
      case states::SCORING:

        bottom_stage_->move_voltage(12000);
        middle_stage_->move_voltage(12000);
        top_stage_->move_voltage(12000);
        break;
      case states::STOPPED:
        bottom_stage_->set_brake_mode_all(pros::MotorBrake::coast);
        middle_stage_->set_brake_mode_all(pros::MotorBrake::hold);
        top_stage_->set_brake_mode_all(pros::MotorBrake::coast);

        bottom_stage_->brake();
        middle_stage_->brake();
        top_stage_->brake();
        break;
        // case states::JUST_THE_FUCKING_BOTTOM:
        //   bottom_stage_->move_voltage(12000);
        //   break;
      case states::STORE_TOP:
        bottom_stage_->move_voltage(12000);
        middle_stage_->move_voltage(12000);
        top_stage_->move_voltage(-8000);
        break;
    }
  }
};