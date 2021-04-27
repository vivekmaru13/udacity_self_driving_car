#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
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
  double max_s = 6945.554;

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

  // Start on lane 1 
  int lane = 1;

  // Reference velocity (mph)
  double reference_vel = 0.0; 

  // Maxiumum Velocity allowed
  const double max_val_allowed = 49.0;

  // Safe distance to maintain
  const int safe_margin_maintain = 30;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &reference_vel,
               &max_val_allowed, &safe_margin_maintain]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          // Total number of points in previous path
          int previous_path_size = previous_path_x.size();

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // Setting the current "s" value to last executed point's "s"
          if (previous_path_size > 0)
          {
            car_s = end_path_s;
          }

          bool lane_change_required = false;
          bool left_lane_is_not_safe = false;
          bool right_lane_is_not_safe = false;

          for ( int i = 0; i < sensor_fusion.size(); i++ )
          {
            double d = sensor_fusion[i][6];

            // Find out target vehicle's lane
            int target_car_lane;
            if ( d > 0 && d < 4 )
            {
              target_car_lane = 0; 
            }
            else if ( d > 4 && d < 8 )
            {
              target_car_lane = 1;
            }
            else if ( d > 8 && d < 12 )
            {
              target_car_lane = 2;
            }

            // Calculate targel car speed
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double target_car_speed = sqrt(vx*vx + vy*vy);
            double target_car_s = sensor_fusion[i][5];

            // Determine target car's "s" position at the end of current cycle/end point
            target_car_s += ((double)previous_path_size*0.02*target_car_speed);
            
            // Check if we need to lane change or not.
            if (target_car_lane == lane && target_car_s > car_s && (target_car_s - car_s) < safe_margin_maintain)
            {
              lane_change_required = true;
            }

            // Check where the target vehicle is and is it safe to change lanes
            if (target_car_lane - lane < 0)
            {
              // target vehicle is in left lane and checking if it is safe to go there
              if (target_car_s> (car_s - safe_margin_maintain) && target_car_s < (car_s + safe_margin_maintain))
              {
                left_lane_is_not_safe = true ;
              }
            }
            else if (target_car_lane - lane == 1 )
            {
              // target vehicle is in right lane and checking if it is safe to go there
              if (target_car_s> (car_s - safe_margin_maintain) && target_car_s < (car_s + safe_margin_maintain))
              {
                right_lane_is_not_safe = true; 
              }
            }

          }

          // Doing the actual lane changing.
          if (lane_change_required)
          {
            if (!left_lane_is_not_safe && lane > 0)
            {
              lane--;
            }
            else if (!right_lane_is_not_safe && lane < 2)
            {
              lane++;
            }
            else
            {
              // if no lane change is possible, just decrease the velocity to avoid collision.
              reference_vel -= 0.224;
            }
          }
          else
          {
            // if nothing is in front of the ego vehicle, and speed is low, increase it.
            if (reference_vel < max_val_allowed)
            {
              reference_vel += 0.224;
            }
          }
          
          
          // point vectors for spline path
           vector<double> pts_x;
           vector<double> pts_y;

          // define references for ego state from last cycle/last points
           double reference_ego_x = car_x;
           double reference_ego_y = car_y;
           double reference_ego_yaw = deg2rad(car_yaw);
          
		      if (previous_path_size < 2)
             // We don't have enough amount of points in the previous_path, so we will find the last executed 
             // point from the previous_path and add it to pts with current location
           {
              double previous_car_x = car_x - cos(reference_ego_yaw); 
              double previous_car_y = car_y - sin(reference_ego_yaw);

              pts_x.push_back(previous_car_x);
              pts_x.push_back(car_x); // remember to do the different way
             
              pts_y.push_back(previous_car_y);
              pts_y.push_back(car_y);
            
           }
          else
            // We have enough points in the previous_path, so we will use last and second last points
            // from the previous_path
          {
             reference_ego_x = previous_path_x[previous_path_size - 1]; // x of the last point in previous_path
             reference_ego_y = previous_path_y[previous_path_size - 1]; // y of the last point in previous_path

             double previous_reference_ego_x = previous_path_x[previous_path_size - 2]; // x of the second last point in previous_path
             double previous_reference_ego_y = previous_path_y[previous_path_size - 2]; // y of the second last point in previous_path

            
             pts_x.push_back(previous_reference_ego_x);
             pts_x.push_back(reference_ego_x);
            
             pts_y.push_back(previous_reference_ego_y);
             pts_y.push_back(reference_ego_y);

            // update ego state yaw
            reference_ego_yaw = atan2(reference_ego_y - previous_reference_ego_y, reference_ego_x - previous_reference_ego_x);
          }
          
          // 3 more waypoints at 30, 60 and 90 meters using "s" value
          vector<double> wp0 = getXY(car_s + 30, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp1 = getXY(car_s + 60, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> wp2 = getXY(car_s + 90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          pts_x.push_back(wp0[0]);
          pts_x.push_back(wp1[0]);
          pts_x.push_back(wp2[0]);

          pts_y.push_back(wp0[1]);
          pts_y.push_back(wp1[1]);
          pts_y.push_back(wp2[1]);
          
          // transofrming the points
          for (int i = 0; i < pts_x.size(); i++)
          {
            double shift_x = pts_x[i] - reference_ego_x;
            double shift_y = pts_y[i] - reference_ego_y;

            pts_x[i] = shift_x*cos(0 - reference_ego_yaw) - shift_y*sin(0 - reference_ego_yaw);
            pts_y[i] = shift_x*sin(0 - reference_ego_yaw) + shift_y*cos(0 - reference_ego_yaw);
          }
		  
          // create a spline object
          tk::spline spline_obj;
          
          // set (x,y) points to the spline.
          spline_obj.set_points(pts_x, pts_y);
          
          // Next X and Y values vectors
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // push points from previous cycles
          for (int i = 0; i < previous_path_size; i++)
          {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }
          
          // Calculate how to break up spline points to travel at reference velocity
          double target_spacing_x = 30.0;
          double target_spacing_y = spline_obj(target_spacing_x);
          double target_dist = sqrt((target_spacing_x * target_spacing_x) + (target_spacing_y * target_spacing_y));

          double x_add_on = 0.0;

          for (size_t i = 1; i <= 50 - previous_path_x.size(); ++i) {
            
            // Find number of points for speed regulations
            double N = target_dist / (0.02 * reference_vel / 2.24);
            double x_point = x_add_on + target_spacing_x / N;
            double y_point = spline_obj(x_point);

            x_add_on = x_point;

            double x_car = x_point;
            double y_car = y_point;

            // Rotate back into previous coordinate system
            x_point =reference_ego_x + (x_car * cos(reference_ego_yaw) - y_car * sin(reference_ego_yaw));
            y_point = reference_ego_y + (x_car * sin(reference_ego_yaw) + y_car * cos(reference_ego_yaw));

            // Final Push
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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