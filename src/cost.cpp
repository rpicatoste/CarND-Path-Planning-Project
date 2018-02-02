//#include "point.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

#include "cost.h"


//TODO: change weights for cost functions.
extern float REACH_GOAL;
extern float EFFICIENCY;

/*
Here we have provided two possible suggestions for cost functions, but feel free to use your own!
The weighted cost over all cost functions is computed in calculate_cost. See get_helper_data
for details on how useful helper data is computed.
*/


float goal_distance_cost(	const Point & vehicle,
							const std::vector<Point> & trajectory,
							const std::map<int, std::vector<Point>> & predictions,
							std::map<std::string, float> & data) {

    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    */
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
    } else {
        cost = 1;
    }
    return cost;

    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    */
/*    //TODO: Implement goal_distance_cost
	float d_distance = abs(	vehicle.lane - vehicle.goal_lane);
	float s_distance = abs(	vehicle.s - vehicle.goal_s);

	float distance = sqrt( d_distance*d_distance + 20.0*20.0*s_distance*s_distance );

	float cost = distance/100.0;

	std::cout << "Distance: lane: " << d_distance << "/frontal: " << s_distance << ". ";
	std::cout << "Total distance: " << distance << ". Cost: " << cost << std::std::endl;

    return cost ;
*/
}

float inefficiency_cost(const Point & vehicle, const std::vector<Point> & trajectory, const std::map<int, std::vector<Point>> & predictions, std::map<std::string, float> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have slower traffic.
    */

    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = vehicle.target_speed;
    }

    float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_speed;
    }

    float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;

    return cost;
}

float lane_speed(const std::map<int, std::vector<Point>> & predictions, int lane) {
    /*
    All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    */
    for (std::map<int, std::vector<Point>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Point vehicle = it->second[0];
        if (vehicle.lane == lane && key != -1) {
            return vehicle.v;
        }
    }
    //Found no vehicle in the lane
    return -1.0;
}

float calculate_cost(const Point & vehicle,
					 const std::map<int, std::vector<Point>> & predictions,
					 const std::vector<Point> & trajectory)
{
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    std::map<std::string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    //Add additional cost functions here.
    std::vector< std::function<float(   const Point & ,
										const std::vector<Point> &,
										const std::map<int, std::vector<Point>> &,
										std::map<std::string, float> &) > >
									cf_list = {goal_distance_cost, inefficiency_cost};
    std::vector<float> weight_list = {REACH_GOAL, EFFICIENCY};

    for (int i = 0; i < cf_list.size(); i++) {
        float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }


	std::cout << "Total cost: " << cost << std::endl;

    return cost;

}

std::map<std::string, float> get_helper_data(const Point & vehicle, const std::vector<Point> & trajectory, const std::map<int, std::vector<Point>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: +/- 1 from the current lane if the vehicle is planning or executing a lane change.
    final_lane: The lane of the vehicle at the end of the trajectory. The lane is unchanged for KL and PLCL/PLCR trajectories.
    distance_to_goal: The s distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
    std::map<std::string, float> trajectory_data;
    Point trajectory_last = trajectory[1];
    float intended_lane;

    if (trajectory_last.state.compare("PLCL") == 0) {
        intended_lane = trajectory_last.lane + 1;
    } else if (trajectory_last.state.compare("PLCR") == 0) {
        intended_lane = trajectory_last.lane - 1;
    } else {
        intended_lane = trajectory_last.lane;
    }

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;
}

