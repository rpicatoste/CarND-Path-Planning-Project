#ifndef COST_H
#define COST_H



#include "point.h"


float calculate_cost(const Point & vehicle,
					const std::map<int, std::vector<Point>> & predictions,
					const std::vector<Point> & trajectory);

float goal_distance_cost(const Point & vehicle,  const std::vector<Point> & trajectory,  const std::map<int, std::vector<Point>> & predictions, std::map<std::string, float> & data);

float inefficiency_cost(const Point & vehicle, const std::vector<Point> & trajectory, const std::map<int, std::vector<Point>> & predictions, std::map<std::string, float> & data);

float lane_speed(const std::map<int, std::vector<Point>> & predictions, int lane);

std::map<std::string, float> get_helper_data(const Point & vehicle, const std::vector<Point> & trajectory, const std::map<int, std::vector<Point>> & predictions);

#endif
