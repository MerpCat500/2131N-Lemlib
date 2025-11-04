#include "2131N/systems/chassis.hpp"

#include "lemlib/chassis/chassis.hpp"

void Chassis::moveToRelativePose(
    const lemlib::Pose& deltaPose, int timeout, lemlib::MoveToPoseParams p, bool async)
{
  lemlib::Pose targetPose = this->getPose() + deltaPose;
  this->moveToPose(targetPose.x, targetPose.y, targetPose.theta, timeout, p, async);
}

void Chassis::moveToPointAsPose(
    const lemlib::Pose& point, int timeout, lemlib::MoveToPointParams p, bool async)
{
  this->lemlib::Chassis::moveToPoint(point.x, point.y, timeout, p, async);
}

void Chassis::moveToRelativePoint(
    const lemlib::Pose& deltaPoint, int timeout, lemlib::MoveToPointParams p, bool async)
{
  lemlib::Pose targetPoint = this->getPose() + deltaPoint;
  this->lemlib::Chassis::moveToPoint(targetPoint.x, targetPoint.y, timeout, p, async);
}

void Chassis::moveToRelativePoint(
    float x, float y, int timeout, lemlib::MoveToPointParams p, bool async)
{
  lemlib::Pose targetPoint = this->getPose() + lemlib::Pose(x, y, 0);
  this->lemlib::Chassis::moveToPoint(targetPoint.x, targetPoint.y, timeout, p, async);
}

void Chassis::turnToRelativeHeading(
    float deltaHeading, int timeout, lemlib::TurnToHeadingParams p, bool async)
{
  this->turnToHeading(this->getPose().theta + deltaHeading, timeout, p, async);
}

lemlib::Pose Chassis::fromPolar(float r, float theta, bool radians)
{
  if (!radians) { theta = theta * (M_PI / 180.0); }
  return lemlib::Pose(r * std::sin(theta), r * std::cos(theta), 0);
}
