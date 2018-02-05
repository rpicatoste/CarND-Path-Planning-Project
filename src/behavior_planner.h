
#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>

#include "sensor_fusion_point.h"
#include "vehicle.h"

class BehaviorPlanner {


public:
	std::vector<double> map_waypoints_x;
	std::vector<double> map_waypoints_y;
	std::vector<double> map_waypoints_s;

	BehaviorPlanner(
		std::vector<double> map_waypoints_x,
		std::vector<double> map_waypoints_y,
		std::vector<double> map_waypoints_s)
	{
		this->map_waypoints_x = map_waypoints_x;
		this->map_waypoints_y = map_waypoints_y;
		this->map_waypoints_s = map_waypoints_s;
	}

  	std::vector<Vehicle> plan_next_position(
			Vehicle &car,
			std::vector<Vehicle> previous_path,
			Vehicle end_path,
			std::vector<SensorFusionPoint> sensor_fusion_points);

  	std::vector<Vehicle> convert_raw_waypoints_to_simulator_waypoints(
  			Vehicle &car,
			Vehicle reference_point,
  			std::vector<Vehicle> next_waypoints_raw,
			std::vector<Vehicle> previous_path);

    std::vector<double> getXY(	double s, double d);

};



#endif 
