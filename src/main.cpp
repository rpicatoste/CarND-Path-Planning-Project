#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include <unistd.h>
#include "spline.h"
#include "road.h"
#include "point.h"
#include "test.h"
#include "helpers.h"


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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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


  // Have a reference velocity to target
  double ref_vel = 0.0; // mph
  // Start in lane 1
  int lane = 1;
  // Point tal = Point(1,2,3,4,5,6);


  h.onMessage([&ref_vel, &lane, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            Point car = Point(json_msg[1]["x"], json_msg[1]["y"], json_msg[1]["s"], json_msg[1]["d"], json_msg[1]["yaw"], json_msg[1]["speed"]);

          	// Previous path data given to the Planner
          	auto previous_path_x = json_msg[1]["previous_path_x"];
          	auto previous_path_y = json_msg[1]["previous_path_y"];

            int prev_size = previous_path_x.size();

            vector<Point> previous_path;
            for( int ii = 0; ii<prev_size; ii++){
              previous_path.push_back( Point(previous_path_x[ii], previous_path_y[ii]) );
            }



          	// Previous path's end s and d values 
            Point end_path = Point(0.0, 0.0, json_msg[1]["end_path_s"], json_msg[1]["end_path_d"]);

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = json_msg[1]["sensor_fusion"];

            
            if(prev_size > 0){
              car.s = end_path.s;
            }

            bool too_close = false;

            // Find ref_v to use
            for(int ii = 0; ii < sensor_fusion.size(); ii++){
              // Car is in my lane
              float d = sensor_fusion[ii][6];
              if(d < (2 + 4.0*lane + 2) && d > (2 + 4*lane - 2)){
                double vx = sensor_fusion[ii][3];
                double vy = sensor_fusion[ii][4];
                double check_speed = sqrt( vx*vx + vy*vy );
                double check_car_s = sensor_fusion[ii][5];

                // If using previous points can project s value out 
                check_car_s += ((double)prev_size*0.02*check_speed);
                // Check s values greater than mine and s gap
                if((check_car_s > car.s) && ((check_car_s - car.s) < 30)){
                  
                  too_close = true;
                  if(lane > 0){
                    lane = 0;
                  }

                }
              }
            }

            if(too_close){
              ref_vel -= 0.224;
            }
            else if(ref_vel < 49.5){
              ref_vel += 0.224;
            }

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // Create very spaced (30m) waypoints, then interpolate with spline.
            vector<Point> pts;


            Point ref = Point();
            ref.x = car.x;
            ref.y = car.y;
            ref.yaw_deg = car.yaw_deg;
            ref.yaw_rad = car.yaw_rad;
            ref.print();


            if(prev_size < 2){
              // Use 2 points that make the path tangent to the car.
              Point prev_car = Point();
              prev_car.x = car.x - cos(car.yaw_rad);
              prev_car.y = car.y - sin(car.yaw_rad);              

              pts.push_back(prev_car);
              pts.push_back(car);              

            }
            // use the previous path's end point as starting reference
            else{
              // Redefine
              ref.x = previous_path_x[prev_size-1];
              ref.y = previous_path_y[prev_size-1];

              Point ref_prev = Point();
              ref_prev.x = previous_path_x[prev_size-2];
              ref_prev.y = previous_path_y[prev_size-2];
              ref.yaw_rad = atan2(ref.y - ref_prev.y, ref.x - ref_prev.x );

              // Use 2 points that make the path tangent to the previous path's end point
              pts.push_back(ref_prev);
              pts.push_back(ref);              

            }

            Point next_wp0 = Point( getXY(car.s + 30,
                                          (2+4*lane),
                                          map_waypoints_s,
                                          map_waypoints_x,
                                          map_waypoints_y) ); 
            Point next_wp1 = Point( getXY(car.s + 60,
                                          (2+4*lane),
                                          map_waypoints_s,
                                          map_waypoints_x,
                                          map_waypoints_y) );
            Point next_wp2 = Point( getXY(car.s + 90,
                                          (2+4*lane),
                                          map_waypoints_s,
                                          map_waypoints_x,
                                          map_waypoints_y) );

            pts.push_back(next_wp0);            
            pts.push_back(next_wp1);
            pts.push_back(next_wp2);


            for(int ii = 0; ii < pts.size(); ii++){
              Point shift = Point();
              shift = pts[ii] - ref;
              pts[ii] = shift.rotate_by_angle(-ref.yaw_rad);
            }

            // Create a spline
            tk:: spline my_spline;

            // Set (x,y) points to the spline
            my_spline.set_points( Point::get_vector_x_from_list(pts), 
                                  Point::get_vector_y_from_list(pts));

            // Define the actual (x,y) points we will use for the planner
            vector<Point> next_vals;
            
            // Start with all of the previous path points from last time
            for (int ii = 0; ii < previous_path_x.size(); ii++){
              Point new_point = Point(previous_path_x[ii], previous_path_y[ii]);
              next_vals.push_back(new_point);
            }

            // Calculate hot to break up spline points so that we travel at our desired reference velocity
            Point target = Point();
            target.x = 30.0;
            target.y = my_spline(target.x);

            double x_add_on = 0;

            // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
            for( int ii = 1; ii <= 50-previous_path_x.size(); ii++ ){
              double N = (target.module()/(0.02*ref_vel/2.24)); // div by 2.24 for MPH

              Point point = Point();
              point.x = x_add_on + (target.x)/N;
              point.y = my_spline(point.x);

              x_add_on = point.x;

              // Rotates back to normal after rotating it earlier
              point = point.rotate_by_angle(ref.yaw_rad);
              point = point + ref;

              next_vals.push_back(point);

            }

            json msgJson;

          	msgJson["next_x"] = Point::get_vector_x_from_list(next_vals);
          	msgJson["next_y"] = Point::get_vector_y_from_list(next_vals);

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
