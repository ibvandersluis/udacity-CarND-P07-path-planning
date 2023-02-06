#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/QR"
#include "nlohmann/json.hpp"
#include "path_planning/path_planning.hpp"
#include "spline.h"
#include "udacity/math.hpp"
#include "udacity/uws.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  auto map_waypoints_x = vector<double>{};
  auto map_waypoints_y = vector<double>{};
  auto map_waypoints_s = vector<double>{};
  auto map_waypoints_dx = vector<double>{};
  auto map_waypoints_dy = vector<double>{};

  // Waypoint map to read from
  auto map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  // double max_s = 6945.554;

  auto in_map_ = std::ifstream{map_file_, std::ifstream::in};

  auto line = string{};
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.emplace_back(x);
    map_waypoints_y.emplace_back(y);
    map_waypoints_s.emplace_back(s);
    map_waypoints_dx.emplace_back(d_x);
    map_waypoints_dy.emplace_back(d_y);
  }

  // Lane 0 = left, Lane 1 = middle, Lane 2 = right
  auto target_lane = int{1};
  auto speed_limit_mph = 50.0;
  auto target_speed_mph = 0.0;

  h.onMessage(
    [&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
     &target_lane, &target_speed_mph, &speed_limit_mph](
      uWS::WebSocket<uWS::SERVER> ws, char * data, size_t length, uWS::OpCode /*opCode*/) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = udacity::uws::has_data(data);

        if (!s.empty()) {
          auto j = json::parse(s);

          auto event = j[0].get<string>();

          if (event == "telemetry") {
            // j[1] is the data JSON object

            // Main car's localization Data
            auto car_x = double{j[1]["x"]};
            auto car_y = double{j[1]["y"]};
            auto car_s = double{j[1]["s"]};
            // double car_d = j[1]["d"];
            auto car_yaw = double{j[1]["yaw"]};
            // double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            auto end_path_s = double{j[1]["end_path_s"]};
            // double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side
            //   of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            auto prev_size = previous_path_x.size();

            auto start_s = (prev_size > 0) ? end_path_s : car_s;
            auto reduce_speed = false;

            auto car_ahead = false;
            auto car_left = false;
            auto car_right = false;

            for (auto & check_car_data : sensor_fusion) {
              auto d = double{check_car_data[6]};

              auto vx = double{check_car_data[3]};
              auto vy = double{check_car_data[4]};
              auto check_car_s = double{check_car_data[5]};
              auto check_car_speed = sqrt(vx * vx + vy * vy);

              check_car_s += ((double)prev_size * 0.02 * check_car_speed);
              auto check_car_lane = int{-1};

              if (d > 0 && d < 4) {
                check_car_lane = 0;
              } else if (d > 4 && d < 8) {
                check_car_lane = 1;
              } else if (d > 8 && d < 12) {
                check_car_lane = 2;
              }

              auto lane_diff = check_car_lane - target_lane;

              switch (lane_diff) {
                case 0:
                  // Vehicle is in our lane
                  if (check_car_s > start_s && check_car_s - start_s < 30) {
                    car_ahead = true;
                    reduce_speed = true;
                  }
                  break;
                case 1:
                  // Vehicle is in the lane on the right
                  if (abs(check_car_s - start_s) < 30) {
                    car_right = true;
                  }
                  break;
                case -1:
                  // Vehicle is in the lane on our left
                  if (abs(check_car_s - start_s) < 30) {
                    car_left = true;
                  }
                  break;
                default:
                  // Car is not in an adjacent lane
                  break;
              }
            }

            if (car_ahead) {
              // Attempt lange change if slower vehicle ahead
              // Prioritize passing on the left and change lanes if safe
              if (target_lane > 0 && !car_left) {
                target_lane -= 1;
              } else if (target_lane < 2 && !car_right) {
                target_lane += 1;
              }
            }

            if (reduce_speed) {
              target_speed_mph -= 0.25;
            } else if (target_speed_mph < (speed_limit_mph - 0.5)) {
              target_speed_mph += 0.25;
            }

            auto target_speed_mps = udacity::math::mph_to_mps(target_speed_mph);
            auto dist_inc = path_planning::mps_to_step_dist(target_speed_mps);

            auto wide_points_x = vector<double>{};
            auto wide_points_y = vector<double>{};

            auto ref_x = double{};
            auto ref_y = double{};
            auto ref_yaw = double{};

            if (prev_size < 2) {
              // Define reference as the car's current state
              ref_x = car_x;
              ref_y = car_y;
              ref_yaw = udacity::math::deg2rad(car_yaw);

              // Add two points going behind the vehicle
              auto prev_car_x = car_x - cos(car_yaw);
              auto prev_car_y = car_y - sin(car_yaw);

              wide_points_x.emplace_back(prev_car_x);
              wide_points_x.emplace_back(car_x);

              wide_points_y.emplace_back(prev_car_y);
              wide_points_y.emplace_back(car_y);
            } else {
              // Define reference as the last point in the previous path
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              // Define second-to-last point in previous path
              auto ref_x_prev = double{previous_path_x[prev_size - 2]};
              auto ref_y_prev = double{previous_path_y[prev_size - 2]};

              // Calculate angle between the two points
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Add last two points from previous path
              wide_points_x.emplace_back(ref_x_prev);
              wide_points_x.emplace_back(ref_x);

              wide_points_y.emplace_back(ref_y_prev);
              wide_points_y.emplace_back(ref_y);
            }

            // In Frenet, add evenly spaced (30m) coordinates ahead of reference
            for (int i = 1; i <= 3; ++i) {
              auto next_wp = path_planning::get_xy(
                start_s + 30 * i, 2 + 4 * target_lane, map_waypoints_s, map_waypoints_x,
                map_waypoints_y);

              wide_points_x.emplace_back(next_wp[0]);
              wide_points_y.emplace_back(next_wp[1]);
            }

            // Transform angle from global reference to car reference
            for (size_t i = 0; i < wide_points_x.size(); ++i) {
              auto shift_x = wide_points_x[i] - ref_x;
              auto shift_y = wide_points_y[i] - ref_y;

              wide_points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              wide_points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            auto spline = tk::spline{};

            spline.set_points(wide_points_x, wide_points_y);

            auto next_x_vals = vector<double>{};
            auto next_y_vals = vector<double>{};

            // Start with all previous points
            for (size_t i = 0; i < previous_path_x.size(); ++i) {
              next_x_vals.emplace_back(previous_path_x[i]);
              next_y_vals.emplace_back(previous_path_y[i]);
            }

            // Break up spline into points of appropriate distance
            auto target_x = 30.0;
            auto target_y = spline(target_x);
            auto target_dist = sqrt(target_x * target_x + target_y * target_y);

            auto x_add_on = 0.0;

            // Fill in the rest of the points (will always be 50)
            for (size_t i = 1; i < 50 - previous_path_x.size(); ++i) {
              auto N = (target_dist / dist_inc);
              auto x_point = x_add_on + target_x / N;
              auto y_point = spline(x_point);

              x_add_on = x_point;

              auto x_ref = x_point;
              auto y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.emplace_back(x_point);
              next_y_vals.emplace_back(y_point);
            }

            json msgJson;

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\"," + msgJson.dump() + "]";

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }  // end "telemetry" if
        } else {
          // Manual driving
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }  // end websocket if
    });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> /*ws*/, uWS::HttpRequest /*req*/) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection(
    [&h](uWS::WebSocket<uWS::SERVER> ws, int /*code*/, char * /*message*/, size_t /*length*/) {
      ws.close();
      std::cout << "Disconnected" << std::endl;
    });

  auto port = int{4567};
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
