#ifndef COST_H
#define COST_H
#include "vehicle.h"

#include <tuple>
#include <vector>
#include <map>

std::tuple<float, float, float, float, float> calculate_cost(
		const Vehicle & vehicle,
		const std::map<int, std::vector<Vehicle>> & predictions,
		const std::vector<Vehicle> & trajectory);

float goal_distance_cost(	const Vehicle & vehicle,
							const std::vector<Vehicle> & trajectory,
							const std::map<int, std::vector<Vehicle>> & predictions,
							std::map<std::string, float> & data);

float goal_lane_cost(	const Vehicle & vehicle,
					 	const std::vector<Vehicle> & trajectory,
						const std::map<int, std::vector<Vehicle>> & predictions,
						std::map<std::string, float> & data);

float inefficiency_cost(const Vehicle & vehicle,
						const std::vector<Vehicle> & trajectory,
						const std::map<int, std::vector<Vehicle>> & predictions,
						std::map<std::string, float> & data);


std::map<std::string, float> get_helper_data(const Vehicle & vehicle,
										const std::vector<Vehicle> & trajectory,
										const std::map<int, std::vector<Vehicle>> & predictions);

void print_costs(	std::vector <float> &costs,
					std::vector <float> &goal_distance_costs,
					std::vector <float> &lane_distance_costs,
					std::vector <float> &inefficiency_costs,
					std::vector <float> &penalizations,
					int best_idx,
					std::vector<std::string> states);

#endif
