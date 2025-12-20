#pragma once

#include <algorithm>
struct Point
{
  double x, y;

  Point operator+(const Point& other) const { return Point{x + other.x, y + other.y}; }

  Point& operator+=(const Point& other)
  {
    x += other.x;
    y += other.y;

    x = std::clamp(x, 0.0, 144.0);
    y = std::clamp(y, 0.0, 144.0);

    return *this;
  }

  Point operator-(const Point& other) const { return Point{x - other.x, y - other.y}; }

  Point& operator-=(const Point& other)
  {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  bool operator==(const Point& other) const { return (x == other.x) && (y == other.y); }
  bool operator!=(const Point& other) const { return (x != other.x) || (y != other.y); }

  Point operator*(const double scalar) const { return Point{x * scalar, y * scalar}; }
};