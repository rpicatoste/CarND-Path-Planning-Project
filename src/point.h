
#ifndef POINT_H
#define POINT_H

#include <vector>
#include <string>
#include <map>
#include <iostream>

#include "sensor_fusion_point.h"

class Point{

public:
	double x;
	double y;

	double s;
	double d;
	double yaw_deg;
	double yaw_rad;

	double v;
	double a;


	int lane;
	float ref_vel;
	int preferred_buffer = 6; // impacts "keep lane" behavior.

	int goal_lane;
	float target_speed; // es lo mismo que ref_vel??
	float goal_s;

	std::map<std::string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

	std::string state = "KL";
	std::vector<std::string> successor_states(void);
	std::vector<Point> choose_next_state(std::map<int, std::vector<Point>> predictions);
	std::vector<Point> generate_trajectory(std::string state, std::map<int, std::vector<Point>> predictions);
	std::vector<Point> constant_speed_trajectory();
	float position_at(int t);
	std::vector<Point> keep_lane_trajectory(std::map<int, std::vector<Point>> predictions);
	bool get_vehicle_ahead(std::map<int, std::vector<Point>> predictions, int lane, Point & rVehicle);
	bool get_vehicle_behind(std::map<int, std::vector<Point>> predictions, int lane, Point & rVehicle);
	std::vector<float> get_kinematics(std::map<int, std::vector<Point>> predictions, int lane);
	std::vector<Point> lane_change_trajectory(std::string state, std::map<int, std::vector<Point>> predictions);
	std::vector<Point> prep_lane_change_trajectory(std::string state, std::map<int, std::vector<Point>> predictions);

	// Point(const Point &point);

	Point();
	Point(std::vector<double> xy);
	Point(double x, double y);
	Point(double x, double y, double s, double d);
	Point(double x, double y, double s, double d, double yaw, double speed);
	Point(int lane, float s, float v, float a, std::string state="CS");
	Point(SensorFusionPoint other_vehicle);

	virtual ~Point();

	void print(std::string text = "");

	double distance(Point p1);
	double distance(Point p1, Point p2);

	Point& operator=(const Point& other_point){
		if (this != &other_point) { // self-assignment check expected
	        this->x = other_point.x;
			this->y = other_point.y;
			this->s = other_point.s;
			this->d = other_point.d;
			this->yaw_deg = other_point.yaw_deg;
			this->yaw_rad = other_point.yaw_rad;
			this->v = other_point.v;

	    }
	    return *this;
	}


	Point operator+(const Point& p1){
		Point sum = Point();
		sum.x = p1.x + this->x;
		sum.y = p1.y + this->y;

		return sum; 
	}


	Point operator-(const Point& p1){
		Point sum = Point();
		sum.x = this->x - p1.x;
		sum.y = this->y - p1.y;

		return sum; 
	}

	Point rotate_by_angle(double angle);

	double module(void);

	std::string to_string(void);
	std::string relative_to_string(const Point& other_point);

	static std::vector<double> get_vector_x_from_list(std::vector<Point> &list){
		std::vector<double> new_list;
		for(std::vector<Point>::iterator p_point = list.begin(); p_point != list.end(); p_point++){
			new_list.push_back(p_point->x);
		}
		return new_list;
	}

	static std::vector<double> get_vector_y_from_list(std::vector<Point> &list){
		std::vector<double> new_list;
		for(std::vector<Point>::iterator p_point = list.begin(); p_point != list.end(); p_point++){
			new_list.push_back(p_point->y);
		}
		return new_list;
	}
};

std::ostream& operator<<(std::ostream& os, const Point& obj);

#endif 
