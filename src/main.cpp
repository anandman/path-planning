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

// how far, in waypoints, do we want to look ahead for other cars and/or ourselves
#define LOOK_AHEAD 50     // at 50mph, we will visit 50 waypoints in 1s, assuming 0.02s/waypoint
// length of average car in meters, to be used to calculate safe following distance
#define CAR_LENGTH 4.8

// convert miles/hour to meters/sec
#define miph2mps(v) (v * 5280 * 12 * 0.0254 / 3600)
// convert meters/sec to miles/hour
#define mps2miph(v) (v * 3600 / (5280 * 12 * 0.0254))
// convert meters/sec to waypoint distance based on simulator visiting waypoints every 0.02s
#define mps2wpdist(v) (v * 0.02)
// convert waypoint distance to meters/sec based on simulator visiting waypoints every 0.02s
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

  int tgt_lane = -1;
  double last_car_s = -1;
  double total_distance = 0;

  h.onMessage([&map_waypoints, &tgt_lane, &total_distance, &last_car_s, &max_s](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          // calculate and print total distance covered
          if (last_car_s < 0) {
            last_car_s = car_s;  // startup condition
          }
          double distance_since_last = car_s - last_car_s;
          if (distance_since_last > 0) {
            total_distance += distance_since_last;
          } else if (distance_since_last < 0 && -distance_since_last > max_s/2) {
            // assume we wrapped around the map
            total_distance += max_s + distance_since_last;
          }
          last_car_s = car_s;
          cout.setf(ios::fixed, ios::floatfield); // set fixed floating format
          cout.precision(2);                      // for fixed format, two decimal places
          cout << "\rtotal distance driven: " << setw(5) << (total_distance/(5280 * 12 * 0.0254)) << " miles" << flush;

          if (tgt_lane < 0) {
            // guess car's target lane (+0.5 is to ensure proper rounding before casting)
            tgt_lane = (int)(car_d - 2 + 0.5) / 4;
          }

          // target speed in mph (goal is to ensure we are below speed limit of 50mph)
          double ref_v = 49.5;

          // save off car reference for future use/modification
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          //cout << "prev path size: " << previous_path_size << endl;
          //cout << "prev path X: " << previous_path_x << endl;
          //cout << "prev path Y: " << previous_path_y << endl;
          //cout << "target lane: " << tgt_lane << endl;
          //cout << "end S/D: " << end_path_s << ", " << end_path_d << endl;
          //cout << "car X/Y/S/D/Y: " << car_x << ", " << car_y << ", " << car_s << ", " << car_d << ", " << car_yaw << endl;

          if (previous_path_size >= 2) {
            // base car reference on previous path rather than current car position
            ref_x = previous_path_x[previous_path_size-1];
            ref_y = previous_path_y[previous_path_size-1];
            double ref_x_prev = previous_path_x[previous_path_size-2];
            double ref_y_prev = previous_path_y[previous_path_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

            // set desired car speed based on previous path
            car_speed = mps2miph(wpdist2mps(sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)
                    +(ref_y-ref_y_prev)*(ref_y-ref_y_prev))));
          }

          //cout << "car speed: " << miph2mps(car_speed) << " m/s" << endl;

          // calculate safe following distance as one car length for every 10mph we're driving
          double safe_distance = (car_speed / 10) * CAR_LENGTH;
          double closest_distance = 1000;
          bool pass_traffic = false;
          bool traffic_on_left = false;
          bool traffic_on_right = false;
          //cout << "safe distance: " << safe_distance << endl;

          // scan other cars around us
          for (int i=0; i < sensor_fusion.size(); i++) {
            int traffic_id = sensor_fusion[i][0];
            double traffic_x = sensor_fusion[i][1];
            double traffic_y = sensor_fusion[i][2];
            double traffic_vx = sensor_fusion[i][3];
            double traffic_vy = sensor_fusion[i][4];
            double traffic_speed = sqrt(traffic_vx*traffic_vx+traffic_vy*traffic_vy);
            double traffic_s = sensor_fusion[i][5];
            double traffic_d = sensor_fusion[i][6];
            double traffic_distance = traffic_s - car_s;

            // TODO: we really should go slower than traffic on our left, but leave that for version 2.0
            // check if traffic is in same lane
            if ((traffic_d > tgt_lane*4) && (traffic_d < tgt_lane*4+4)) {
              // check if traffic in front of us is within safe following distance
              //cout << "traffic " << traffic_id << " in same lane (d=" << traffic_d << "): " << traffic_distance << "m ahead of us traveling at " << traffic_speed << "m/s" << endl;
              if ((traffic_distance > 0) && (traffic_distance <= safe_distance) && (traffic_distance < closest_distance)) {
                //cout << "found traffic " << traffic_id << " within safe distance traveling at " << traffic_speed << "mph" << endl;
                // set safe distance to this traffic's distance so that we can see if there are any others even closer
                closest_distance = traffic_distance;

                // set ref speed to traffic in front of us
                ref_v = miph2mps(traffic_speed);
                pass_traffic = true;
              }
            }

            // check if traffic is present in lane to left
            if ((tgt_lane > 0) && (traffic_d > (tgt_lane-1)*4) && (traffic_d < (tgt_lane-1)*4+4)) {
              // TODO: use traffic's speed/acceleration instead of using safe distance for traffic "behind" us
              if ((traffic_distance > -safe_distance) && (traffic_distance < safe_distance)) {
                traffic_on_left = true;
              }
            }

            // check if traffic is present in lane to right
            if ((tgt_lane < 2) && (traffic_d > (tgt_lane+1)*4) && (traffic_d < (tgt_lane+1)*4+4)) {
              // TODO: use traffic's speed/acceleration instead of using safe distance for traffic "behind" us
              if ((traffic_distance > -safe_distance) && (traffic_distance < safe_distance)) {
                traffic_on_right = true;
              }
            }

            // TODO: need to check and recover if traffic we are passing changes into our new target la
          }

          // pass traffic if too slow in front of us or change lanes to the right if we're in the passing lane
          if (pass_traffic || tgt_lane == 0) {
            if (!traffic_on_left && tgt_lane > 0) {
              // pass on left
              tgt_lane -= 1;
            } else if (!traffic_on_right && tgt_lane < 2) {
              // pass on right
              // TODO: really shouldn't do this but the other cars suck and go slowly everywhere
              tgt_lane += 1;
            }
          }

          // create spline of immediate vicinity
          vector<double> spl_x, spl_y;
          if (previous_path_size < 2) {
            // estimate and add previous car position
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            spl_x.push_back(prev_car_x);
            spl_y.push_back(prev_car_y);

            // fix end path so we don't assume 0
            end_path_s = car_s;
            //end_path_d = car_d;

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

          // add next 3 waypoints beyond previous path to spline
          vector<double> xy0, xy1, xy2;
          xy0 = getXY(end_path_s+30, 2+4*tgt_lane, map_waypoints);
          xy1 = getXY(end_path_s+60, 2+4*tgt_lane, map_waypoints);
          xy2 = getXY(end_path_s+90, 2+4*tgt_lane, map_waypoints);
          spl_x.push_back(xy0[0]);
          spl_y.push_back(xy0[1]);
          spl_x.push_back(xy1[0]);
          spl_y.push_back(xy1[1]);
          spl_x.push_back(xy2[0]);
          spl_y.push_back(xy2[1]);

          // ensure spline's X is sorted increasing
          for (int i = 0; i < spl_x.size(); i++) {
            // we do this by shifting all points into car reference space (ie, car yaw = 0)
            double shift_x = (spl_x[i]-ref_x);
            double shift_y = (spl_y[i]-ref_y);
            spl_x[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
            spl_y[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
          }

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
          double x_inc = 0;
          for (int i=1; i <= LOOK_AHEAD-previous_path_size; i++) {
            if (ref_v > car_speed) {
              // if we're below target speed, then accelerate a little bit
              car_speed += mps2miph(0.1);
            } else if (ref_v < car_speed) {
              // if we're above target speed, the decelerate a little bit
              car_speed -= mps2miph(0.1);
            }

            // N is the number of points required to acheive our desired speed
            double N = tgt_dist / (mps2wpdist(miph2mps(car_speed)));
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
          cout << "next path X: ";
          for (auto x: next_x_vals)
            cout << x << ", ";
          cout << endl << "next path Y: ";
          for (auto y: next_y_vals)
            cout << y << ", ";
          cout << endl;
          */

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
