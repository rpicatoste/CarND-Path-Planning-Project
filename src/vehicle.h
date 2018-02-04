
#ifndef POINT_H
#define POINT_H

#include <iostream>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "sensor_fusion_point.h"

class Vehicle{

public:
	double x;
	double y;

	double s;
	double d;
	double yaw_deg;
	double yaw_rad;

	float current_velocity;
	double velocity;
	double a;

	int reference_lane;
	int preferred_buffer = 6; // impacts "keep lane" behavior.

	// Porting
	int goal_lane;
	float target_speed;
	float goal_s;
	int lanes_available;
	float max_acceleration;


	int get_current_lane(void);
	float get_desired_s(void);

	std::map<std::string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}, {"KL", 0}, {"CS", 0}};

	std::string state = "KL";
	std::vector<std::string> successor_states(void);
	std::vector<Vehicle> choose_next_state(std::map<int, std::vector<Vehicle>> predictions);
	std::vector<Vehicle> generate_trajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions);
	std::vector<Vehicle> constant_speed_trajectory();
	float position_at(int t);
	std::vector<Vehicle> keep_lane_trajectory(std::map<int, std::vector<Vehicle>> predictions);
	bool get_vehicle_ahead(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
	bool get_vehicle_behind(std::map<int, std::vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
	std::vector<float> get_kinematics(std::map<int, std::vector<Vehicle>> predictions, int lane);
	std::vector<Vehicle> lane_change_trajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions);
	std::vector<Vehicle> prep_lane_change_trajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions);
	void print_info(void);
	void increment(int dt);
    std::vector<Vehicle> generate_predictions(int horizon=2);
    void realize_next_state(std::vector<Vehicle> trajectory);
    void configure(std::vector<int> road_data);
	// Point(const Point &point);

	Vehicle();
	Vehicle(std::vector<double> xy);
	Vehicle(double x, double y);
	Vehicle(double x, double y, double s, double d);
	Vehicle(double x, double y, double s, double d, double yaw, double speed);
	Vehicle(int lane, float s, float v, float a, std::string state="CS");
	Vehicle(SensorFusionPoint other_vehicle);

	virtual ~Vehicle();

	void print(std::string text = "");

	double distance(Vehicle p1);
	double distance(Vehicle p1, Vehicle p2);

	Vehicle& operator=(const Vehicle& other_point){
		if (this != &other_point) { // self-assignment check expected
	        this->x = other_point.x;
			this->y = other_point.y;
			this->s = other_point.s;
			this->d = other_point.d;
			this->yaw_deg = other_point.yaw_deg;
			this->yaw_rad = other_point.yaw_rad;
			this->velocity = other_point.velocity;

	    }
	    return *this;
	}


	Vehicle operator+(const Vehicle& p1){
		Vehicle sum = Vehicle();
		sum.x = p1.x + this->x;
		sum.y = p1.y + this->y;

		return sum; 
	}


	Vehicle operator-(const Vehicle& p1){
		Vehicle sum = Vehicle();
		sum.x = this->x - p1.x;
		sum.y = this->y - p1.y;

		return sum; 
	}

	Vehicle rotate_by_angle(double angle);

	double module(void);

	std::string to_string(void);
	std::string relative_position_to_string(const Vehicle& other_point);

	static std::vector<double> get_vector_x_from_list(std::vector<Vehicle> &list){
		std::vector<double> new_list;
		for(std::vector<Vehicle>::iterator p_point = list.begin(); p_point != list.end(); p_point++){
			new_list.push_back(p_point->x);
		}
		return new_list;
	}

	static std::vector<double> get_vector_y_from_list(std::vector<Vehicle> &list){
		std::vector<double> new_list;
		for(std::vector<Vehicle>::iterator p_point = list.begin(); p_point != list.end(); p_point++){
			new_list.push_back(p_point->y);
		}
		return new_list;
	}
};

std::ostream& operator<<(std::ostream& os, const Vehicle& obj);

#endif 
