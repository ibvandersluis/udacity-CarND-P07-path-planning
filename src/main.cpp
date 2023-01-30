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
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  // double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
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
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage(
    [&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](
      uWS::WebSocket<uWS::SERVER> ws, char * data, size_t length, uWS::OpCode /*opCode*/) {
      // "42" at the start of the message means there's a websocket message event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event
      if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = udacity::uws::has_data(data);

        if (s != "") {
          auto j = json::parse(s);

          string event = j[0].get<string>();

          if (event == "telemetry") {
            // j[1] is the data JSON object

            // Main car's localization Data
            double car_x = j[1]["x"];
            double car_y = j[1]["y"];
            double car_s = j[1]["s"];
            // double car_d = j[1]["d"];
            double car_yaw = j[1]["yaw"];
            // double car_speed = j[1]["speed"];

            // Previous path data given to the Planner
            auto previous_path_x = j[1]["previous_path_x"];
            auto previous_path_y = j[1]["previous_path_y"];
            // Previous path's end s and d values
            // double end_path_s = j[1]["end_path_s"];
            // double end_path_d = j[1]["end_path_d"];

            // Sensor Fusion Data, a list of all other cars on the same side
            //   of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            json msgJson;

            // TODO: define x,y points that the car will visit sequentially every .02 seconds
            auto target_lane = 1;
            auto target_speed_mph = 49.5;
            auto prev_size = previous_path_x.size();

            // auto target_speed_mps = udacity::math::mph_to_mps(target_speed_mph);
            // auto dist_inc = path_planning::mps_to_step_dist(target_speed_mps);

            vector<double> wide_points_x;
            vector<double> wide_points_y;

            double ref_x;
            double ref_y;
            double ref_yaw;

            if (prev_size < 2) {
              // Define reference as the car's current state
              ref_x = car_x;
              ref_y = car_y;
              ref_yaw = udacity::math::deg2rad(car_yaw);

              // Add two points going behind the vehicle
              auto prev_car_x = car_x - cos(car_yaw);
              auto prev_car_y = car_y - sin(car_yaw);

              wide_points_x.push_back(prev_car_x);
              wide_points_x.push_back(car_x);

              wide_points_y.push_back(prev_car_y);
              wide_points_y.push_back(car_y);

            } else {
              // Define reference as the last point in the previous path
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              // Define second-to-last point in previous path
              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];

              // Calculate angle between the two points
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Add last two points from previous path
              wide_points_x.push_back(ref_x_prev);
              wide_points_x.push_back(ref_x);

              wide_points_y.push_back(ref_y_prev);
              wide_points_y.push_back(ref_y);
            }

            // In Frenet, add evenly spaces (30m) coordinates ahead of reference
            for (int i = 1; i <= 3; ++i) {
              auto next_wp = path_planning::get_xy(
                car_s + 30 * i, 2 + 4 * target_lane, map_waypoints_s, map_waypoints_x,
                map_waypoints_y);

              wide_points_x.push_back(next_wp[0]);
              wide_points_y.push_back(next_wp[1]);
            }

            // Transform angle from global reference to car reference
            for (size_t i = 0; i < wide_points_x.size(); ++i) {
              auto shift_x = wide_points_x[i] - ref_x;
              auto shift_y = wide_points_y[i] - ref_y;

              wide_points_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              wide_points_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            tk::spline spline;

            spline.set_points(wide_points_x, wide_points_y);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // Start with all previous points
            for (size_t i = 0; i < previous_path_x.size(); ++i) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            // Break up spline into points of appropriate distance
            auto target_x = 30.0;
            auto target_y = spline(target_x);
            auto target_dist = sqrt(target_x * target_x + target_y * target_y);

            auto x_add_on = 0.0;

            // Fill in the rest of the points (will always be 50)
            for (size_t i = 1; i < 50 - previous_path_x.size(); ++i) {
              auto N = (target_dist / (0.02 * target_speed_mph / 2.24));
              auto x_point = x_add_on + target_x / N;
              auto y_point = spline(x_point);

              x_add_on = x_point;

              auto x_ref = x_point;
              auto y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            // END-TODO

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

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
