#ifndef UDACITY__MATH_HPP
#define UDACITY__MATH_HPP

#include <cmath>

namespace udacity
{

namespace math
{

double deg2rad(double x) { return x * M_PI / 180; }

double rad2deg(double x) { return x * 180 / M_PI; }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

}  // namespace math

}  // namespace udacity

#endif  // UDACITY__MATH_HPP
