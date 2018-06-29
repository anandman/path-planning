#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <iterator>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "waypoint.h"

// convert miles/hour to meters/sec
#define miph2mps(v) (v * 5280 * 12 * 0.0254 / 3600)
// convert meters/sec to miles/hour
#define mps2miph(v) (v * 3600 / (5280 * 12 * 0.0254))
// convert meters/sec to waypoint distance based on what the simulator does
#define mps2wpdist(v) (v * 0.02)
// convert waypoint distance to meters/sec based on what the simulator does
#define wpdist2mps(v) (v / 0.02)

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<Waypoint> map_waypoints, hires_map_waypoints;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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
  	map_waypoints.push_back({.x = x,.y = y,.s = s,.dx = d_x,.dy = d_y});
  }

  // build hires map by interpolating points in-between
  // to go 50mph, taking 0.02s between waypoints, each waypoint would need to be 0.447m apart
  //hires_map_waypoints = getInterpolatedWaypoints(map_waypoints, max_s, mps2wpdist(miph2mps(ref_v)));

  h.onMessage([&map_waypoints](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

        	// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					// path to return
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// size of previous points not yet reached
          int previous_path_size = previous_path_x.size();

          // guess car's target lane
          int tgt_lane = ((int)car_d / 4) - 2;
          tgt_lane = 1;

          // target speed in mph
          double ref_v = 49.5;

          // save off car reference for future use/modification
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          cout << miph2mps(ref_v) << " m/s, " << mps2wpdist(miph2mps(ref_v)) << " m/waypoint" << endl;
          cout << "prev path size: " << previous_path_size << endl;
          cout << "prev path X: " << previous_path_x << endl;
          cout << "prev path Y: " << previous_path_y << endl;
          cout << "target lane: " << tgt_lane << endl;
          cout << "end S/D: " << end_path_s << ", " << end_path_d << endl;
          cout << "car X/Y/S/D/Y: " << car_x << ", " << car_y << ", " << car_s << ", " << car_d << ", " << car_yaw << endl;

          if (previous_path_size < 2) {
            // do nothing right now
          } else {
            // base car reference on previous path rather than current car position
            ref_x = previous_path_x[previous_path_size-1];
            ref_y = previous_path_y[previous_path_size-1];
            double ref_x_prev = previous_path_x[previous_path_size-2];
            double ref_y_prev = previous_path_y[previous_path_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            // snap car location to last previous path point consumed
            car_s = end_path_s;
            car_d = end_path_d;

            // set desired car speed based on previous path
            car_speed = mps2miph(wpdist2mps(sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)
                    +(ref_y-ref_y_prev)*(ref_y-ref_y_prev))));
          }

          // create spline of immediate vicinity
          vector<double> spl_x, spl_y;
          if (previous_path_size < 2) {
            // estimate and add previous car position
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            spl_x.push_back(prev_car_x);
            spl_y.push_back(prev_car_y);

            // add current car position
            spl_x.push_back(car_x);
            spl_y.push_back(car_y);
          } else {
            // add previous++ car position
            spl_x.push_back(previous_path_x[previous_path_size-2]);
            spl_y.push_back(previous_path_y[previous_path_size-2]);

            // add previous car position
            spl_x.push_back(previous_path_x[previous_path_size-1]);
            spl_y.push_back(previous_path_y[previous_path_size-1]);
          }

          // add next 3 waypoints to spline
          vector<double> xy0, xy1, xy2;
          xy0 = getXY(car_s+30, 2+4*tgt_lane, map_waypoints);
          xy1 = getXY(car_s+60, 2+4*tgt_lane, map_waypoints);
          xy2 = getXY(car_s+90, 2+4*tgt_lane, map_waypoints);
          spl_x.push_back(xy0[0]);
          spl_y.push_back(xy0[1]);
          spl_x.push_back(xy1[0]);
          spl_y.push_back(xy1[1]);
          spl_x.push_back(xy2[0]);
          spl_y.push_back(xy2[1]);

          // ensure spline's X is sorted increasing
          cout << "spline: ";
          for (int i = 0; i < spl_x.size(); i++) {
            // we do this by shifting all points into car reference space (ie, car yaw = 0)
            cout << "[" << spl_x[i] << "," << spl_y[i] << "] -> ";
            double shift_x = (spl_x[i]-ref_x);
            double shift_y = (spl_y[i]-ref_y);
            spl_x[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            spl_y[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
            cout << "[" << spl_x[i] << "," << spl_y[i] << "] ";
          }
          cout << endl;

          // create spline from points
          tk::spline tgt_spl;
          tgt_spl.set_points(spl_x,spl_y);

          // add any previous path left over
          for (int i = 0; i < previous_path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // add up to 50 waypoints (at 50mph, this is roughly 1s)
          double tgt_x = 30.0;
          double tgt_y = tgt_spl(tgt_x);
          double tgt_dist = sqrt(tgt_x*tgt_x+tgt_y*tgt_y);
          cout << "target: " << tgt_x << ", " << tgt_y << " - " << tgt_dist << endl;
          double x_inc = 0;
          for (int i=1; i <= 50-previous_path_size; i++) {
            if (ref_v > car_speed) {
              // if we're below target speed, then accelerate
              car_speed += 0.224;
            } else if (ref_v < car_speed) {
              // if we're above target speed, the decelerate
              car_speed -= 0.224;
            }

            // N is the number of points required to acheive our desired speed
            double N = tgt_dist / (mps2wpdist(miph2mps(car_speed)));
            cout << "N = " << N << " (" << mps2wpdist(miph2mps(car_speed)) << ")" << endl;
            double x = x_inc + tgt_x/N;
            double y = tgt_spl(x);
            x_inc = x;

            // shift points back with reference to actual car yaw
            double x_ref = x;
            double y_ref = y;
            x = ref_x + x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
            y = ref_y + x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

          /*
          // get closest target waypoint to car
          int tgt_wpt_idx = NextWaypoint(car_x, car_y, car_yaw, hires_map_waypoints);

          // get next 1s of path (~22m or ~50 waypoints)
          for (int i = tgt_wpt_idx+previous_path_size; i < tgt_wpt_idx+50; i++) {
            // handle end of track
            int adjusted_i = i;
            if (i >= hires_map_waypoints.size()) {
              adjusted_i -= hires_map_waypoints.size();
            }
            vector<double> xy = getXY(hires_map_waypoints[adjusted_i].s, 6.0, hires_map_waypoints);
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }
          */

          cout << "next path X: ";
          for (auto x: next_x_vals)
            cout << x << ", ";
          cout << endl << "next path Y: ";
          for (auto y: next_y_vals)
            cout << y << ", ";
          cout << endl;

          msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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
