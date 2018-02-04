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

	// From the DISTANTE_TO_START_GOING_TO_GOAL_LANE there will be a surge of importance to be in the proper lane.
	float weight_by_distance = DISTANCE_TO_START_GOING_TO_GOAL_LANE / abs(distance);
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
	// Saturate s_distance. Above a certain distance it will not have increasing effect.
	float s_max_value = DISTANCE_TO_START_GOING_TO_GOAL_LANE;
	s_distance = (s_distance > s_max_value) ? s_max_value : s_distance;

	// Normalize
	cost = s_distance/DISTANCE_TO_START_GOING_TO_GOAL_LANE;

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
    float distance;
    float lane_speed ;
    float cost;
    float multiplier_distance_to_goal ;


    distance = abs(data["distance_to_goal"]);
    lane_speed = vehicle.get_lane_speed(predictions, (int)data["intended_lane"]);

    cost = (vehicle.target_speed - lane_speed)/vehicle.target_speed;
    // The further we are, the more important it is to go faster. Then the importance decreases.
    multiplier_distance_to_goal = (distance/DISTANCE_TO_START_GOING_TO_GOAL_LANE);
    multiplier_distance_to_goal = SATURATE(multiplier_distance_to_goal, 10.0);

    cost *= multiplier_distance_to_goal;

    printf("Cost % 6.0f, final lane speed: ----% 3.0f----, multiplier %f\n",
            cost, lane_speed, multiplier_distance_to_goal);

    return cost;
}


/*
Sum weighted cost functions to get total cost for trajectory.
*/
std::tuple<float, float, float, float, float, float> calculate_cost(
		const Vehicle & vehicle,
		const std::map<int, std::vector<Vehicle>> & predictions,
		const std::vector<Vehicle> & trajectory)
{
    std::map<std::string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    float change_lane_bonification, new_goal_distance_cost, new_inefficiency_cost, new_goal_lane_cost, change_lane_penalization, candidate_state_speed;

	new_goal_distance_cost = weight_distance_goal   * goal_distance_cost(vehicle, trajectory, predictions, trajectory_data);
	new_goal_lane_cost     = weight_lane_goal   * goal_lane_cost(vehicle, trajectory, predictions, trajectory_data);
	new_inefficiency_cost  = weight_efficiency * inefficiency_cost(vehicle, trajectory, predictions, trajectory_data);

	change_lane_penalization = (trajectory[1].state.compare("KL")==0) ? 0.0 : LANE_CHANGE_PENALIZATION;

	bool lane_change_started = trajectory[1].state.compare("LCR")==0 || trajectory[1].state.compare("LCL")==0;
	change_lane_bonification = (lane_change_started) ? LANE_CHANGE_BONIFICATION : 0.0;

	candidate_state_speed = vehicle.get_lane_speed(predictions, (int)trajectory_data["intended_lane"]);


    cost = new_inefficiency_cost + new_goal_distance_cost + new_goal_lane_cost + change_lane_penalization + change_lane_bonification;

    return std::make_tuple(cost, new_inefficiency_cost, new_goal_distance_cost, new_goal_lane_cost, change_lane_penalization, candidate_state_speed);
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

    intended_lane = trajectory_last.reference_lane + trajectory_last.lane_direction[trajectory_last.state];

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;

    trajectory_data["intended_lane"] = (float)intended_lane;
    trajectory_data["final_lane"] = (float)trajectory_last.reference_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;
}

void print_costs(	std::vector <float> &costs,
					std::vector <float> &goal_distance_costs,
					std::vector <float> &lane_distance_costs,
					std::vector <float> &inefficiency_costs,
                    std::vector <float> &penalizations,
                    std::vector <float> &lane_speeds,
					int best_idx,
					std::vector<std::string> states)
{
	std::vector<float>::iterator cost;
	std::string txt_states 				= "Possible States (";
	std::string txt_costs 				= "Total cost      (";
	std::string txt_goal_distance_costs = "Goal dist cost  (";
	std::string txt_lane_distance_costs = "Lane dist cost  (";
	std::string txt_inefficiency_costs 	= "Inefficienc cost(";
	std::string txt_penalization     	= "Change Penalty  (";
	std::string txt_lane_speeds      	= "Lane speed      (";
	char aux_cost[20] = "";
	char aux_goal_distance_cost[20] = "";
	char aux_lane_distance_cost[20] = "";
	char aux_inefficiency_cost[20] = "";
	char aux_penalization[20] = "";
	char aux_lane_speeds[20] = "";
	char aux_state[20] = "";
	for(int ii = 0; ii < costs.size(); ii++ ){

		if(best_idx == ii){
			sprintf(aux_cost,  "(% 12.1f)", costs[ii]);
			sprintf(aux_goal_distance_cost,  "(% 12.1f)", goal_distance_costs[ii]);
			sprintf(aux_lane_distance_cost,  "(% 12.1f)", lane_distance_costs[ii]);
			sprintf(aux_inefficiency_cost,  "(% 12.1f)", inefficiency_costs[ii]);
			sprintf(aux_penalization,  "(% 12.1f)", penalizations[ii]);
			sprintf(aux_lane_speeds,  "(% 12.1f)", lane_speeds[ii]);
			sprintf(aux_state, "(% 12s)", states[ii].c_str());
		}
		else{
			sprintf(aux_cost,  " % 12.1f ", costs[ii]);
			sprintf(aux_goal_distance_cost,  " % 12.1f ", goal_distance_costs[ii]);
			sprintf(aux_lane_distance_cost,  " % 12.1f ", lane_distance_costs[ii]);
			sprintf(aux_inefficiency_cost,  " % 12.1f ", inefficiency_costs[ii]);
			sprintf(aux_penalization,  " % 12.1f ", penalizations[ii]);
			sprintf(aux_lane_speeds,  " % 12.1f ", lane_speeds[ii]);
			sprintf(aux_state, " % 12s ", states[ii].c_str());
		}

		txt_costs  += std::string(aux_cost);
		txt_goal_distance_costs  += std::string(aux_goal_distance_cost);
		txt_lane_distance_costs  += std::string(aux_lane_distance_cost);
		txt_inefficiency_costs   += std::string(aux_inefficiency_cost);
		txt_penalization         += std::string(aux_penalization);
		txt_lane_speeds         += std::string(aux_lane_speeds);
		txt_states += std::string(aux_state);
//		std::cout << txt_states[ii] << std::endl;
	}
	txt_costs += ")";
	txt_goal_distance_costs += ")";
	txt_lane_distance_costs += ")";
	txt_inefficiency_costs += ")";
	txt_penalization += ")";
	txt_lane_speeds += ")";
	txt_states += ")";
	std::cout << txt_states << std::endl;
	std::cout << txt_costs << std::endl;
	std::cout << txt_goal_distance_costs << std::endl;
	std::cout << txt_lane_distance_costs << std::endl;
	std::cout << txt_inefficiency_costs << std::endl;
	std::cout << txt_penalization << std::endl;
	std::cout << txt_lane_speeds << std::endl;
}


