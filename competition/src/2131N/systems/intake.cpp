#include "2131N/systems/intake.hpp"

#include <cmath>
#include <utility>

Intake::Intake(
    pros::Motor* top_stage_motor,
    pros::Motor* bottom_stage_motor,
    pros::Motor* storage_motor,
    std::vector<pros::Optical*> color_sensors,
    pros::Controller* controller,
    pros::controller_digital_e_t intake_button,
    pros::controller_digital_e_t outtake_button,
    pros::controller_digital_e_t store_button,
    pros::controller_digital_e_t unstore_button,
    const float red_sort_bound,
    const float blue_sort_bound)
    : bottom_stage_(bottom_stage_motor),
      top_stage_(top_stage_motor),
      storage_(storage_motor),
      color_sensors_(std::move(color_sensors)),
      primary_(controller),
      intake_button_(intake_button),
      outtake_button_(outtake_button),
      store_button_(store_button),
      unstore_button_(unstore_button),
      red_sort_bound_(red_sort_bound),
      blue_sort_bound_(blue_sort_bound),
      color_sort_task_(
          [this]() {
            while (true)
            {
              update();
              pros::delay(10);
            }
          },
          "Color Sort Task")
{
  // Set Color Sensors to update every 10ms instead of every 20ms
  for (auto colorSensor : color_sensors) { colorSensor->set_integration_time(10); }
}

void Intake::teleOp()
{
  // If not actively sorting
  if (!sorting_)
  {
    // If the intake and outtake button are pressed
    if (primary_->get_digital(intake_button_) && primary_->get_digital(outtake_button_))
    {
      // Set the state to score in the middle goal
      setState(IntakeState::SCORE_MIDDLE, StorageState::IGNORE);
    }
    // else if the intake button is pressed
    else if (primary_->get_digital(intake_button_))
    {
      // set the intake to intake
      setState(IntakeState::INTAKE);
    }
    // else if the outtake button is pressed
    else if (primary_->get_digital(outtake_button_))
    {
      // Outtake at half of max voltage (to score in low goal)
      setState(IntakeState::OUTTAKE, StorageState::IGNORE, 6000);
    }
    // Else stop the intake
    else { setState(IntakeState::IDLE); }

    // If the store button is pressed
    if (primary_->get_digital(store_button_))
    {
      // Start storing balls
      setState(IntakeState::IGNORE, StorageState::STORE);
    }
    // If the unstore button is pressed
    else if (primary_->get_digital(unstore_button_))
    {
      // Start releasing balls
      setState(IntakeState::IGNORE, StorageState::UNSTORE);
    }
    // Else stop the storage
    else { setState(IntakeState::IGNORE, StorageState::IDLE); }
  }
}

void Intake::setState(IntakeState intakeState, StorageState storageState, std::int16_t voltage)
{
  // Absolute value voltage (sign will be internally handled)
  voltage = std::abs(voltage);

  // Check all intake states
  switch (intakeState)
  {
    case IntakeState::IDLE:  // Stop the intake
      bottom_stage_->brake();
      top_stage_->brake();
      break;
    case IntakeState::INTAKE:  // Intake the intake
      bottom_stage_->move_voltage(voltage);
      top_stage_->move_voltage(voltage);
      break;
    case IntakeState::SCORE_MIDDLE:  // Score in the middle goal
      // Spin slowly backwards to stop balls from jamming above the middle goal
      top_stage_->move_voltage(-voltage / 4);
      bottom_stage_->move_voltage(voltage);
      break;
    case IntakeState::OUTTAKE:  // Outtake
      bottom_stage_->move_voltage(-voltage);
      top_stage_->move_voltage(-voltage);
      break;
    case IntakeState::IGNORE:  // Ignore
      break;
  }

  // Check all storage states
  switch (storageState)
  {
    case StorageState::IDLE:  // Stop Storage
      storage_->brake();
      break;
    case StorageState::STORE:  // Set to Store
      storage_->move_voltage(voltage);
      break;
    case StorageState::UNSTORE:  // Set to Unstore
      storage_->move_voltage(-voltage);
      break;
    case StorageState::IGNORE:  // Ignore
      break;
  }

  // If the state isn't set to ignore, update the current state
  if (intakeState != IntakeState::IGNORE) { current_intake_state_ = intakeState; }
  if (storageState != StorageState::IGNORE) { current_storage_state_ = storageState; }
}

void Intake::setSortEnabled(bool sort_enabled, bool is_red_team)
{
  color_sort_enabled_ = sort_enabled;
  is_red_team_ = is_red_team;
}

void Intake::update()
{
  // TODO: Be reworked
  if (color_sort_enabled_)
  {
    float reading = 0.0f;
    for (auto colorSensor : color_sensors_)
    {
      colorSensor->set_led_pwm(100);
      pros::c::optical_rgb_s_t rgb = colorSensor->get_rgb();
      reading += rgb.red - rgb.blue;
    }

    reading /= color_sensors_.size();

    if (((reading > red_sort_bound_ && is_red_team_) ||
         (reading < -blue_sort_bound_ && !is_red_team_)) &&
        !sorting_)
    {
      sorting_ = true;

      pre_sort_intake_state_ = current_intake_state_;
      pre_sort_storage_state_ = current_storage_state_;

      if (current_intake_state_ == IntakeState::INTAKE &&
          current_storage_state_ == StorageState::STORE)
      {
        setState(IntakeState::INTAKE, StorageState::IDLE, bottom_stage_->get_voltage());
      }
      else if (
          current_intake_state_ == IntakeState::INTAKE &&
          current_storage_state_ == StorageState::UNSTORE)
      {
        setState(IntakeState::SCORE_MIDDLE, StorageState::UNSTORE, bottom_stage_->get_voltage());
      }
      else if (
          current_intake_state_ == IntakeState::SCORE_MIDDLE &&
          current_storage_state_ == StorageState::UNSTORE)
      {
        setState(IntakeState::INTAKE, StorageState::UNSTORE, bottom_stage_->get_voltage());
      }
      else { sorting_ = false; }
      sorting_ = true;
    }
    else if (
        !((reading > red_sort_bound_ && is_red_team_) ||
          (reading < -blue_sort_bound_ && !is_red_team_)) &&
        sorting_)
    {
      setState(pre_sort_intake_state_, pre_sort_storage_state_, bottom_stage_->get_voltage());
      sorting_ = false;
    }
  }
  else
  {
    for (auto colorSensor : color_sensors_) { colorSensor->set_led_pwm(0); }
    sorting_ = false;
  }
}