#include "autonomous.hpp"
#include "2131N/robot-config.hpp"

void debug(bool is_red_team)
{
  chassis.setPose({0,0,0});
  chassis.moveToPoint(0, 24, 2000, {}, false);

}