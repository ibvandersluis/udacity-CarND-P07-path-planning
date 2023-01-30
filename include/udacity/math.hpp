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

// Convert miles per hour to meters per second
double mph_to_mps(double mph)
{
  auto mps = mph / 2.237;
  return mps;
}

}  // namespace math

}  // namespace udacity

#endif  // UDACITY__MATH_HPP
