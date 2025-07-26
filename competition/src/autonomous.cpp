#include "autonomous.hpp"

#include <sys/_intsup.h>

#include "2131N/robot-config.hpp"
#include "2131N/systems/intake.hpp"
#include "lemlib/chassis/chassis.hpp"

void debug(bool is_red_team)
{
  chassis.setPose({0, 0, 0});
  chassis.moveToPoint(0, 24, 2000, {}, false);
}

void leftSideAWP(bool is_red_team)
{
  // ! Initial
  chassis.setPose({72.0 - 13.25 / 2 - 0.375 - 9, 24 - 7 - 9, 0});

  // * Intake Group of three
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPose(49.0, 47.5, -50, 2000, {.lead = 0.4, .maxSpeed = 80}, false);

  // * Score in middle goals
  chassis.turnToPoint(72.0, 73.0, 800, {}, false);
  chassis.moveToPoint(58, 59, 1000, {}, false);
  intake.setState(Intake::IntakeState::SCORE_MIDDLE, Intake::StorageState::UNSTORE, 12000);
  pros::delay(2000);

  // * Move to second group of three
  chassis.swingToPoint(72, 43, lemlib::DriveSide::LEFT, 800, {.minSpeed = 1}, false);
  chassis.moveToPoint(72, 43, 2000, {}, false);
  intake.setState(Intake::IntakeState::INTAKE, Intake::StorageState::STORE);
  chassis.moveToPose(96, 50, 140, 2000, {.lead = 0.3, .maxSpeed = 70}, false);

  // * Score in bottom middle goal
  chassis.turnToPoint(72.0, 71.0, 800, {}, false);
  chassis.moveToPoint(72.0, 71.0, 1000, {}, false);
  intake.setState(Intake::IntakeState::OUTTAKE, Intake::StorageState::UNSTORE, 6000);
}