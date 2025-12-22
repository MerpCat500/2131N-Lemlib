/**
 * @file chassis.hpp
 * @author Andrew Hilton (2131N)
 * @brief Additional chassis functionality
 * @version 0.1
 * @date 2025-11-04
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <cmath>

#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"

class Chassis : public lemlib::Chassis
{
 public:
  void moveToRelativePose(
      const lemlib::Pose& deltaPose,
      int timeout,
      lemlib::MoveToPoseParams p = {},
      bool async = true);

  void moveToPointAsPose(
      const lemlib::Pose& point, int timeout, lemlib::MoveToPointParams p = {}, bool async = true);

  void moveToRelativePoint(
      const lemlib::Pose& deltaPoint,
      int timeout,
      lemlib::MoveToPointParams p = {},
      bool async = true);

  void moveToRelativePoint(float x, float y, int timeout, lemlib::MoveToPointParams p, bool async);

  void turnToRelativeHeading(
      float deltaHeading, int timeout, lemlib::TurnToHeadingParams p = {}, bool async = true);

  static lemlib::Pose fromPolar(float r, float theta, bool radians = false);

  void tank_with_dead_zone(
      double left_speed, double right_speed, double dead_zone, bool drive_curve = false);
};
