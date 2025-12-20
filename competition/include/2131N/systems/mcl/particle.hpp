#pragma once

#include "point.hpp"

class Particle
{
 private:
  Point position;
  double weight;

 public:
  // constructible with initial position and weight
  Particle(Point init_pos = {0.0, 0.0}, double init_weight = 0.0)
      : position(init_pos), weight(init_weight)
  {
  }

  void set_position(const Point &p) { position = p; }

  void move(Point delta_position) { position += delta_position; }

  Point get_position() const { return position; }

  double get_weight() const { return weight; }

  void set_weight(double new_weight) { weight = new_weight; }

  void normalize(double total_weight)
  {
    if (total_weight > 0) { weight /= total_weight; }
  }
};
