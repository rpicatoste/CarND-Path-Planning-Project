
#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>

#include "sensor_fusion_point.h"
#include "vehicle.h"

class BehaviorPlanner {

  public:

  	std::vector<Vehicle> plan_next_position(Vehicle &car, 
                                          std::vector<Vehicle> previous_path, 
                                          Vehicle end_path,
                                          std::vector<SensorFusionPoint> sensor_fusion_points,
                                          std::vector<double> map_waypoints_x,
                                          std::vector<double> map_waypoints_y,
                                          std::vector<double> map_waypoints_s);

  	std::vector<Vehicle> convert_raw_waypoints_to_simulator_waypoints(
  			Vehicle &car,
			Vehicle reference_point,
  			std::vector<Vehicle> next_waypoints_raw,
			std::vector<Vehicle> previous_path);

    std::vector<double> getXY(
    		double s,
			double d,
			const std::vector<double> &maps_s,
			const std::vector<double> &maps_x,
			const std::vector<double> &maps_y);

/*
  	string ego_rep = " *** ";

  	int ego_key = -1;

  	int num_lanes;

    vector<int> lane_speeds;

    int speed_limit;

    double density;

    int camera_center;

    map<int, Vehicle> vehicles;

    int vehicles_added = 0;

    // Constructor
  	Road(int speed_limit, double traffic_density, vector<int> lane_speeds);

  	// Destructor
  	virtual ~Road();

  	Vehicle get_ego();

  	void populate_traffic();

  	void advance();

  	void display(int timestep);

  	void add_ego(int lane_num, int s, vector<int> config_data);

  	void cull();
*/

};



#endif 
