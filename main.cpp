
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
using std::cout;
using std::endl;

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

  
    
	int current_lane = 1; // start in lane 1 (left most lane
	double ref_vel = 0.0; // reference velocity
	double max_speed = 48.0;
	
	
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


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &current_lane,&ref_vel, &max_speed]
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
		  
		  int prev_size = previous_path_x.size();
		  
		  // Sensor fusion, look for cars in your lane and slow down
		  
		  if (prev_size > 0)
		  {
			  car_s = end_path_s;
		  }
          
		  bool too_close = false;
		  int other_car_lane;
		  double safe_distance = 30.0;
		  double intended_lane_speed = max_speed;
		  double left_lane_cost = 999.0;
		  double right_lane_cost = 999.0;
		  double current_lane_cost = 999.0;
		  current_lane = find_car_lane(car_d);
		  int best_lane = current_lane;
          
		 
		  for (int i=0 ; i<sensor_fusion.size() ; i++)
		  {
			  float d = sensor_fusion[i][6];
			  //cout<<" car # "<< i <<" = lateral dist = " << d << endl;
			  double vx = sensor_fusion[i][3];
			  double vy = sensor_fusion[i][4];
			  double check_speed = sqrt(vx*vx + vy*vy);
			  double check_car_s = sensor_fusion[i][5];
			  // find where this car will be in the future
			  check_car_s += ((double)prev_size * 0.02 * check_speed); // number of points remaining * 20ms * that car's speed
			  
			  other_car_lane = find_car_lane(d);
			  
			  if (other_car_lane == current_lane && check_car_s > car_s && (check_car_s-car_s) <safe_distance) // current lane not open, calculate cost of this lane and look for other lanes
			  {
				  Struct current_lane_prop = laneCost(current_lane, sensor_fusion, car_s, ref_vel, safe_distance);
				  intended_lane_speed = current_lane_prop.v_ahead;
				  current_lane_cost = current_lane_prop.cost;
				 double best_lane_cost = current_lane_cost;
				 best_lane = current_lane; 
				 cout << "current LC = " << current_lane_cost << " : ";
				 if (current_lane !=0) // my car not in left most lane
				 {
					 Struct current_lane_prop = laneCost(current_lane - 1, sensor_fusion, car_s, ref_vel, safe_distance);
					 intended_lane_speed = current_lane_prop.v_ahead;
					 left_lane_cost = current_lane_prop.cost;
					 bool left_lane_open = laneOpenCheck(current_lane-1, sensor_fusion, car_s, ref_vel, safe_distance);
					 if (left_lane_open && left_lane_cost < best_lane_cost)
					 {
						 best_lane = current_lane - 1;
						 best_lane_cost = left_lane_cost;
					 }
				 }
				 cout << " left LC = " << left_lane_cost << " : ";
				if (current_lane !=2) // my car not in right most lane
				 {
					 Struct current_lane_prop = laneCost(current_lane + 1, sensor_fusion, car_s, ref_vel, safe_distance);
					 intended_lane_speed = current_lane_prop.v_ahead;
					 right_lane_cost = current_lane_prop.cost;
					 bool right_lane_open = laneOpenCheck(current_lane+1, sensor_fusion, car_s, ref_vel, safe_distance);
					 if (right_lane_open && right_lane_cost < best_lane_cost)
					 {
						 best_lane = current_lane + 1;
						 best_lane_cost = right_lane_cost;
					 }
				 }
				 cout << "right LC = " << right_lane_cost << " : ";
				 if (best_lane == current_lane)
				 {
					 too_close = true;
					 intended_lane_speed = check_speed;
				 }
				 cout << " best LC = " << best_lane_cost << endl;
				 //else
				   //  max_speed = 48;
			  }
				  
			  
		  }
			  
			  
		  if (ref_vel < max_speed && !too_close)
			  ref_vel += 13 * 0.02 * 2.23; // normal_acceleration = 10 m/sec2 * .02 ms programming rate * 80% to be in acceptable range * m/sec to mph
          //else if (too_close)
			//  ref_vel -= 0.225; // normal_acceleration = 5m/sec2 so for every 20ms the or 0.25m/sec2
		  else if (ref_vel == intended_lane_speed && too_close)
			  ref_vel = intended_lane_speed;
		  else if (ref_vel > intended_lane_speed && too_close)
			  ref_vel -= 13 * 0.02 * 2.23; // normal_acceleration = 10 m/sec2 * .02 ms programming rate * 80% to be in acceptable range * m/sec to mph
		  
			  //else
			//  ref_vel = max_speed;
		  
		  
		  
		  
		  // End of behavior planning
		  
		  vector<double> ptsx;
		  vector<double> ptsy;
		  
		  // Car's starting point or previous path's end point
		  double ref_x = car_x;
		  double ref_y = car_y;
		  double ref_yaw = deg2rad(car_yaw);
		  
		  // if previous path is almost empty
		  if(prev_size<2)
		  {
			  double prev_car_x = car_x - car_x*cos(car_yaw);
			  double prev_car_y = car_y - car_y*sin(car_yaw);
			  ptsx.push_back(prev_car_x);
			  ptsx.push_back(car_x);
			  
			  ptsy.push_back(prev_car_y);
			  ptsy.push_back(car_y);
		  }
		  // else pick the last 2 points from the previous path and define reference state
		  else
		  {
			  ref_x = previous_path_x[prev_size-1];
			  ref_y = previous_path_y[prev_size-1];
			  
			  double ref_x_prev = previous_path_x[prev_size-2];
			  double ref_y_prev = previous_path_y[prev_size-2];
			  ref_yaw = atan2( (ref_y-ref_y_prev),(ref_x - ref_x_prev ));
			  
			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);
			  
			  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);
		  }
		  
		  // adding 3 more points at 30 meters from the last reference points
		  vector<double> next_wp0 = getXY(car_s + 40 , (2+4*best_lane) ,  map_waypoints_s ,  map_waypoints_x , map_waypoints_y);
		  vector<double> next_wp1 = getXY(car_s + 80 , (2+4*best_lane) ,  map_waypoints_s ,  map_waypoints_x , map_waypoints_y);
		  vector<double> next_wp2 = getXY(car_s + 120 , (2+4*best_lane) ,  map_waypoints_s ,  map_waypoints_x , map_waypoints_y);
				
		  ptsx.push_back(next_wp0[0]);
		  ptsx.push_back(next_wp1[0]);
		  ptsx.push_back(next_wp2[0]);
		  
		  ptsy.push_back(next_wp0[1]);
		  ptsy.push_back(next_wp1[1]);
		  ptsy.push_back(next_wp2[1]);
		  
		  // converting car local coordinates to globaal coordinates
          for (int i = 0 ; i<ptsx.size(); ++i)
		  {
			  // shift car reference angle to 0 degrees
			  double shift_x = ptsx[i] - ref_x; 
			  double shift_y = ptsy[i] - ref_y;
			  
			  ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
			  ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
		  }
		  
		  // create a spline
		  tk::spline s;
		  
		  s.set_points(ptsx,ptsy);
		  

          vector<double> next_x_vals;
          vector<double> next_y_vals;
		  
		  // adding previous path points to the new path points sequence
		  for (int i = 0; i<previous_path_x.size(); ++i)
		  {
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
			  
		  }
		  
		  // break up points for next 50 meters
		  double target_x = 50.0;
		  double target_y = s(target_x);
		  double target_dist = sqrt(target_x*target_x + target_y*target_y);
		  
		  double x_add_on = 0;
		  
		  for(int i = 0; i <= 50-previous_path_x.size(); ++i)
		  {
			  double N = target_dist / (.02*ref_vel/2.24);
			  double x_point = x_add_on + (target_x/N);
			  double y_point = s(x_point);
			  
			  x_add_on = x_point;
			  
			  double x_ref = x_point;
			  double y_ref = y_point;
			  
			  // rotate back to normal after rotating it earlier
			  x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
			  y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));
			  
			  // adding car location points to the future trajectory points
			  x_point += ref_x;
			  y_point += ref_y;
			  
			  next_x_vals.push_back(x_point);
			  next_y_vals.push_back(y_point);
		  }
		  
		  
		  

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           *
		   double ideal_vspd = 48; //mph
		   double time_to_update_frame = 0.02; // 20 ms
		   double ideal_vspd_m_sec = ideal_vspd * 0.447; // m/sec
		   
		   double meters_per_frame = ideal_vspd_m_sec * time_to_update_frame;
		   double dist_inc = meters_per_frame;
		   for (int i = 0; i <50 ; ++i)
		   {
			   double next_s = car_s + (i+1)*dist_inc;
			   double next_d = 6;
			   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			   
			   next_x_vals.push_back(xy[0]);
			   next_y_vals.push_back(xy[1]);
		   }
		   //   */
		
          //END
		  json msgJson;

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