/**
 * @file feild.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2025-11-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <algorithm>
#include <cstdlib>
#include <limits>

#include "point.hpp"

class Field
{
 private:
  // Field Boundaries
  Point field_min;
  Point field_max;

 public:
  Field(Point minimum, Point maximum) : field_min(minimum), field_max(maximum) {}

  double get_distance_to_wall(
      const Point point, const double cached_cos, const double cached_sin) const
  {
    double t1 = std::numeric_limits<double>::infinity();
    double t2 = std::numeric_limits<double>::infinity();
    double t3 = std::numeric_limits<double>::infinity();
    double t4 = std::numeric_limits<double>::infinity();

    if (std::abs(cached_cos) > 1e-9)
    {
      // Calculate Intersections with vertical walls

      double a = (field_max.x - point.x) / cached_cos;
      double b = (field_min.x - point.x) / cached_cos;

      // Check if intersections exist within the field
      if (a >= 0)
      {
        double y_int = point.y + a * cached_sin;
        if (y_int >= field_min.y && y_int <= field_max.y) t1 = a;
      }
      if (b >= 0)
      {
        double y_int = point.y + b * cached_sin;
        if (y_int >= field_min.y && y_int <= field_max.y) t2 = b;
      }
    }

    // Calculate Intersections with horizontal walls
    if (std::abs(cached_sin) > 1e-9)
    {
      double c = (field_max.y - point.y) / cached_sin;
      double d = (field_min.y - point.y) / cached_sin;

      // Check if intersections exist within the field
      if (c >= 0)
      {
        double x_int = point.x + c * cached_cos;
        if (x_int >= field_min.x && x_int <= field_max.x) t3 = c;
      }

      if (d >= 0)
      {
        double x_int = point.x + d * cached_cos;
        if (x_int >= field_min.x && x_int <= field_max.x) t4 = d;
      }
    }

    // Return the minimum positive intersection distance
    return std::min({t1, t2, t3, t4});
  }

  Point get_max_point() const { return field_max; }
  Point get_min_point() const { return field_min; }
};