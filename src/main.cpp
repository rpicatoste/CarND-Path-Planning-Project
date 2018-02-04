#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <thread>
#include <unistd.h>
#include <uWS/uWS.h>
#include <vector>

#include "json.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "behavior_planner.h"
#include "helpers.h"
#include "main_constants.h"
#include "road.h"
#include "sensor_fusion_point.h"
#include "test.h"
#include "vehicle.h"

using namespace std;



// for convenience
using json = nlohmann::json;

constexpr double pi() { return M_PI; }

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

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];


		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

vector<double> *global_p_map_waypoints_x;
vector<double> *global_p_map_waypoints_y;
vector<double> *global_p_map_waypoints_s;


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
  double max_s = GOAL_S;

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
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  global_p_map_waypoints_x = &map_waypoints_x;
  global_p_map_waypoints_y = &map_waypoints_y;
  global_p_map_waypoints_s = &map_waypoints_s;

  // Start in lane 1
  int lane = START_LANE;
  Vehicle car = Vehicle();
  car.reference_lane = lane;

	vector<int> car_config = {	SPEED_LIMIT_MPH,
								NUM_LANES,
								GOAL_S,
								GOAL_LANE,
								MAX_ACCELERATION_METER_S2};

	car.configure(car_config);

  h.onMessage([&car, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto json_msg = json::parse(s);
        
        string event = json_msg[0].get<string>();
        
        if (event == "telemetry") {
          // json_msg[1] is the data JSON object
          
        	// Main car's localization Data
            car.x = json_msg[1]["x"];
            car.y = json_msg[1]["y"];
            car.s = json_msg[1]["s"];
            car.d = json_msg[1]["d"];
            car.yaw_rad = json_msg[1]["yaw"];
            car.current_velocity = json_msg[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = json_msg[1]["previous_path_x"];
          	auto previous_path_y = json_msg[1]["previous_path_y"];

            int prev_size = previous_path_x.size();

            vector<Vehicle> previous_path;
            for( int ii = 0; ii<prev_size; ii++){
              previous_path.push_back( Vehicle(previous_path_x[ii], previous_path_y[ii]) );
            }

          	// Previous path's end s and d values 
            Vehicle end_path = Vehicle(0.0, 0.0, json_msg[1]["end_path_s"], json_msg[1]["end_path_d"]);

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = json_msg[1]["sensor_fusion"];

            // Get sensor fusion data
            vector<SensorFusionPoint> sensor_fusion_points;
            for(int ii = 0; ii < sensor_fusion.size(); ii++){
                SensorFusionPoint new_point;
                new_point.d = sensor_fusion[ii][6];
                new_point.vx = sensor_fusion[ii][3];
                new_point.vy = sensor_fusion[ii][4];
                new_point.speed = sqrt( new_point.vx*new_point.vx + new_point.vy*new_point.vy );
                new_point.s = sensor_fusion[ii][5];

                sensor_fusion_points.push_back(new_point);
            }

            BehaviorPlanner behavior_planner = BehaviorPlanner();
            vector<Vehicle> next_vals = behavior_planner.plan_next_position(car, 
                                                                          previous_path, 
                                                                          end_path, 
                                                                          sensor_fusion_points,
                                                                          map_waypoints_x,
                                                                          map_waypoints_y,
                                                                          map_waypoints_s);

            json msgJson;

          	msgJson["next_x"] = Vehicle::get_vector_x_from_list(next_vals);
          	msgJson["next_y"] = Vehicle::get_vector_y_from_list(next_vals);

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }

      //usleep(100000);
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
