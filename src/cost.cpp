#include "cost.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

#include "vehicle.h"
#include "main_constants.h"

// Change weights for cost functions (in main.cpp).
float const weight_distance_goal = WEIGHT_DISTANCE_GOAL;
float const weight_lane_goal     = WEIGHT_LANE_GOAL;
float const weight_efficiency    = WEIGHT_EFFICIENCY;


/*
Cost due to the distance from the car's lane to the goal lane.
Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
*/
float goal_lane_cost(	const Vehicle & vehicle,
					 	const std::vector<Vehicle> & trajectory,
						const std::map<int, std::vector<Vehicle>> & predictions,
						std::map<std::string, float> & data)
{
    float cost;
    float distance      = data["distance_to_goal"];
    float intended_lane = data["intended_lane"];
    float final_lane    = data["final_lane"];

	float lane_distance = abs(	intended_lane+final_lane - 2*vehicle.goal_lane);

	// From 50 meters (the numerator) there will be a surge of imporante to be in the proper lane (or we will miss it).
	float weight_by_distance = 50.0 / abs(distance);
	lane_distance *= weight_by_distance;

	cost = lane_distance;
    return cost ;
}

/*
Cost due to the distance to the goal s.
*/
float goal_distance_cost(	const Vehicle & vehicle,
							const std::vector<Vehicle> & trajectory,
							const std::map<int, std::vector<Vehicle>> & predictions,
							std::map<std::string, float> & data)
{
    float cost;

	float s_distance = abs(	vehicle.s - vehicle.goal_s);
	// Saturate s_distance. Above 100 meters it will not have increasing effect.
	float s_max_value = 100.0;
	s_distance = (s_distance > s_max_value) ? s_max_value : s_distance;

	// Normalize
	cost = s_distance/100.0;

    return cost ;
}

/*
Cost becomes higher for trajectories with intended lane and final lane that have slower traffic.
*/
float inefficiency_cost(const Vehicle & vehicle,
						const std::vector<Vehicle> & trajectory,
						const std::map<int, std::vector<Vehicle>> & predictions,
						std::map<std::string, float> & data)
{
    float distance      = abs(data["distance_to_goal"]);

    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = vehicle.target_speed;
    }

    float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_speed;
    }

    float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;

    // The further we are, the more important it is to go faster. At 200 meters the speed starts to matter less.
    cost *= (distance/200.0);

    return cost;
}

float lane_speed(const std::map<int, std::vector<Vehicle>> & predictions, int lane)
{
    /*
    All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    */
	std::map<int, std::vector<Vehicle>>::const_iterator it;
    for (it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.desired_lane == lane && key != -1) {
            return vehicle.velocity;
        }
    }
    //Found no vehicle in the lane
    return -1.0;
}

/*
Sum weighted cost functions to get total cost for trajectory.
*/
std::tuple<float, float, float, float> calculate_cost(	const Vehicle & vehicle,
														const std::map<int, std::vector<Vehicle>> & predictions,
														const std::vector<Vehicle> & trajectory)
{
    std::map<std::string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    float new_goal_distance_cost, new_inefficiency_cost, new_goal_lane_cost;

	new_goal_distance_cost = weight_distance_goal   * goal_distance_cost(vehicle, trajectory, predictions, trajectory_data);
	new_goal_lane_cost     = weight_lane_goal   * goal_lane_cost(vehicle, trajectory, predictions, trajectory_data);
	new_inefficiency_cost  = weight_efficiency * inefficiency_cost(vehicle, trajectory, predictions, trajectory_data);

    cost = new_inefficiency_cost + new_goal_distance_cost + new_goal_lane_cost;

    return std::make_tuple(cost, new_inefficiency_cost, new_goal_distance_cost, new_goal_lane_cost);
}

/*
Generate helper data to use in cost functions:
 - intended_lane: +/- 1 from the current lane if the vehicle is planning or executing a lane change.
 - final_lane: The lane of the vehicle at the end of the trajectory. The lane is unchanged for KL and PLCL/PLCR trajectories.
 - distance_to_goal: The s distance of the vehicle to the goal.

Note that intended_lane and final_lane are both included to help differentiate between planning and executing
a lane change in the cost functions.
*/
std::map<std::string, float> get_helper_data(	const Vehicle & vehicle,
												const std::vector<Vehicle> & trajectory,
												const std::map<int, std::vector<Vehicle>> & predictions) // Not used
{
    std::map<std::string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    int intended_lane;

    intended_lane = trajectory_last.desired_lane + trajectory_last.lane_direction[trajectory_last.state];

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane = trajectory_last.desired_lane;
    trajectory_data["intended_lane"] = (float)intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;
}

void print_costs(	std::vector <float> &costs,
					std::vector <float> &goal_distance_costs,
					std::vector <float> &lane_distance_costs,
					std::vector <float> &inefficiency_costs,
					int best_idx,
					std::vector<std::string> states)
{
	std::vector<float>::iterator cost;
	std::string txt_states 				= "Possible States (";
	std::string txt_costs 				= "Total cost      (";
	std::string txt_goal_distance_costs = "Goal dist cost  (";
	std::string txt_lane_distance_costs = "Lane dist cost  (";
	std::string txt_inefficiency_costs 	= "Inefficienc cost(";
	char aux_cost[20] = "";
	char aux_goal_distance_cost[20] = "";
	char aux_lane_distance_cost[20] = "";
	char aux_inefficiency_cost[20] = "";
	char aux_state[20] = "";
	for(int ii = 0; ii < costs.size(); ii++ ){

		if(best_idx == ii){
			sprintf(aux_cost,  "(% 12.1f)", costs[ii]);
			sprintf(aux_goal_distance_cost,  "(% 12.1f)", goal_distance_costs[ii]);
			sprintf(aux_lane_distance_cost,  "(% 12.1f)", lane_distance_costs[ii]);
			sprintf(aux_inefficiency_cost,  "(% 12.1f)", inefficiency_costs[ii]);
			sprintf(aux_state, "(% 12s)", states[ii].c_str());
		}
		else{
			sprintf(aux_cost,  " % 12.1f ", costs[ii]);
			sprintf(aux_goal_distance_cost,  " % 12.1f ", goal_distance_costs[ii]);
			sprintf(aux_lane_distance_cost,  " % 12.1f ", lane_distance_costs[ii]);
			sprintf(aux_inefficiency_cost,  " % 12.1f ", inefficiency_costs[ii]);
			sprintf(aux_state, " % 12s ", states[ii].c_str());
		}

		txt_costs  += std::string(aux_cost);
		txt_goal_distance_costs  += std::string(aux_goal_distance_cost);
		txt_lane_distance_costs  += std::string(aux_lane_distance_cost);
		txt_inefficiency_costs   += std::string(aux_inefficiency_cost);
		txt_states += std::string(aux_state);
//		std::cout << txt_states[ii] << std::endl;
	}
	txt_costs += ")";
	txt_goal_distance_costs += ")";
	txt_lane_distance_costs += ")";
	txt_inefficiency_costs += ")";
	txt_states += ")";
	std::cout << txt_states << std::endl;
	std::cout << txt_costs << std::endl;
	std::cout << txt_goal_distance_costs << std::endl;
	std::cout << txt_lane_distance_costs << std::endl;
	std::cout << txt_inefficiency_costs << std::endl;
}


