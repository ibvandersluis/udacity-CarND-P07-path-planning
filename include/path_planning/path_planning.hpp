#ifndef PATH_PLANNING__PATH_PLANNING_H
#define PATH_PLANNING__PATH_PLANNING_H

#include <cmath>
#include <string>
#include <vector>

#include "udacity/math.hpp"

namespace path_planning
{

// Calculate closest waypoint to current x, y position
int get_closest_waypoint(
  double x, double y, const std::vector<double> & maps_x, const std::vector<double> & maps_y)
{
  double closestLen = 100000;  //large number
  int closest_waypoint = 0;

  for (size_t i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = udacity::math::distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

// Returns next waypoint of the closest waypoint
int get_next_waypoint(
  double x, double y, double theta, const std::vector<double> & maps_x,
  const std::vector<double> & maps_y)
{
  size_t closest_waypoint = get_closest_waypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closest_waypoint];
  double map_y = maps_y[closest_waypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = std::min(2 * M_PI - angle, angle);

  if (angle > M_PI / 2) {
    ++closest_waypoint;
    if (closest_waypoint == maps_x.size()) {
      closest_waypoint = 0;
    }
  }

  return closest_waypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> get_frenet(
  double x, double y, double theta, const std::vector<double> & maps_x,
  const std::vector<double> & maps_y)
{
  int next_wp = get_next_waypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = udacity::math::distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = udacity::math::distance(center_x, center_y, x_x, x_y);
  double centerToRef = udacity::math::distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += udacity::math::distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += udacity::math::distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> get_xy(
  double s, double d, const std::vector<double> & maps_s, const std::vector<double> & maps_x,
  const std::vector<double> & maps_y)
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

// Calculate distance per 20ms timestep (50 Hz)
double mps_to_step_dist(double mps)
{
  auto step_dist = mps / 50;
  return step_dist;
}

}  // namespace path_planning

#endif  // PATH_PLANNING__PATH_PLANNING_H
