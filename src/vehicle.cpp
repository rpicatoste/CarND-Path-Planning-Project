#include "vehicle.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <math.h>

#include "behavior_planner.h"
#include "cost.h"
#include "helpers.h"
#include "main_constants.h"


float REACH_GOAL = pow(10, 6);
float EFFICIENCY = pow(10, 5);

Vehicle::Vehicle()
{
	this->velocity = 0.0;
}

Vehicle::Vehicle(std::vector<double> xy)
{
	this->x = xy[0];
	this->y = xy[1];

	this->velocity = 0.0;
}

Vehicle::Vehicle(double x, double y)
{
	this->x = x;
	this->y = y;

	this->velocity = 0.0;
}

Vehicle::Vehicle(double x, double y, double s, double d)
{
	this->x  = x;
	this->y  = y;
	this->s  = s;
	this->d  = d;

	this->velocity = 0.0;
}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw_deg, double speed)
{
	this->x  = x;
	this->y  = y;
	this->s  = s;
	this->d  = d;
	this->yaw_deg = yaw_deg;
	this->yaw_rad = deg2rad(yaw_deg);

	this->velocity = speed;

	this->velocity = 0.0;
}

//class BehaviorPlanner;

Vehicle::Vehicle(SensorFusionPoint other_vehicle, BehaviorPlanner &bp)
{
    this->x = other_vehicle.x;
    this->y = other_vehicle.y;
	this->s  = other_vehicle.s;
	this->d  = other_vehicle.d;
	this->yaw_deg = 0.0;
	this->yaw_rad = deg2rad(yaw_deg);

	this->velocity = other_vehicle.speed;
	this->reference_lane = this->get_current_lane();
}

Vehicle::Vehicle(int lane, float s, float v, float a, std::string state)
{
	this->reference_lane = lane;
	this->s = s;
	this->velocity = v;
	this->a = a;
	this->state = state;

    std::vector<int> null_config = {0, 0, 0, 0};
    configure(null_config);

}

Vehicle::~Vehicle() 
{

}


void Vehicle::print(std::string text)
{
	printf("%sPoint (x: % 7.1f, y: % 7.1f, s: % 7.1f, lane: %d, v: % 2.0f, yaw_deg: % 6.1f).\n",
			text.c_str(), this->x, this->y, this->s, this->get_current_lane(), this->velocity, this->yaw_deg);
}


double Vehicle::module(void)
{
	Vehicle zero = Vehicle(0.0, 0.0);

	return this->distance(zero);
}

double Vehicle::distance(Vehicle p)
{
	return distance(*this, p);
}


double Vehicle::distance(Vehicle p1, Vehicle p2)
{
	return sqrt( (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) );
}

Vehicle Vehicle::rotate_by_angle(double angle)
{
	Vehicle rotated = Vehicle(*this);

	rotated.x = (this->x * cos(angle) - this->y * sin(angle));
	rotated.y = (this->x * sin(angle) + this->y * cos(angle));

	return rotated;
}

std::string Vehicle::to_string(void)
{
    char aux[10] = "";
	std::string txt = "(";
	const char* format = "%5.0f";

    sprintf(aux, format, this->x);
    txt += std::string("x: ") + std::string(aux);

    sprintf(aux, format, this->y);
    txt += std::string(", y: ") + std::string(aux);

    sprintf(aux, format, this->s);
    txt += std::string(", s: ") + std::string(aux);

    sprintf(aux, format, this->d);
    txt += std::string(", d: ") + std::string(aux);

	txt += std::string(")");

	return txt;
}


std::string Vehicle::relative_position_to_string(const Vehicle& my_car)
{
    char aux[10] = "";
	std::string txt = "(";
	const char* format = "%6.1f";

    sprintf(aux, format, this->s - my_car.s);
    txt += std::string("s: ") + std::string(aux);

    txt += std::string(", lane: ") + std::to_string(this->get_current_lane());

    sprintf(aux, format, this->velocity);
    txt += std::string(", v: ") + std::string(aux);

	txt += std::string(")");

	return txt;
}


std::ostream& operator<<(std::ostream& os, const Vehicle& point)
{

    return os << "(x:" << point.x << ", y:" << point.y << "// s:" << point.s << ", d:" << point.d << ")";
}


float Vehicle::get_desired_s(void)
{
	return (this->reference_lane + 0.5)*LANE_WIDTH;
}

int Vehicle::get_current_lane(void)
{
	int current_lane;

	current_lane = (int)std::round((this->d - LANE_WIDTH/2) / LANE_WIDTH);

	return current_lane;
}

/*********************************************************************************/


void Vehicle::print_info()
{
	char s_text[10], v_text[10], a_text[10];
	sprintf(s_text, "% 4d",this->s);
	sprintf(v_text, "% 5.1f",this->velocity);
	sprintf(a_text, "% 4.1f",this->a);
	std::cout << "    Position s:" << s_text << ", lane: " << this->reference_lane;
	std::cout << ", speed:" << v_text << ", acceleration:" << a_text << std::endl;
}

std::vector<Vehicle> Vehicle::choose_next_state(std::map<int, std::vector<Vehicle>> predictions)
{
    /*
    INPUT: A predictions std::map. This is a std::map using vehicle id as keys with predicted
        vehicle trajectories as values. A trajectory is a std::vector of Vehicle objects. The first
        item in the trajectory represents the vehicle at the current timestep. The second item in
        the trajectory represents the vehicle one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next ego vehicle state.

    Functions that will be useful:
    1. successor_states() - Uses the current state to return a std::vector of possible successor states for the finite
       state machine.
    2. generate_trajectory(std::string state, std::map<int, std::vector<Vehicle>> predictions) - Returns a std::vector of Vehicle objects
       representing a vehicle trajectory, given a state and predictions. Note that trajectory vectors
       might have size 0 if no possible trajectory exists for the state.
    3. calculate_cost(Vehicle vehicle, std::map<int, std::vector<Vehicle>> predictions, std::vector<Vehicle> trajectory) - Included from
       cost.cpp, computes the cost for a trajectory.
    */
    std::vector<std::string> possible_succesor_states;
	std::vector<std::vector<Vehicle>> succesor_states_trajectories;
	std::vector <float> costs;
	std::vector <float> goal_distance_costs;
	std::vector <float> lane_distance_costs;
	std::vector <float> inefficiency_costs;
	std::vector <float> penalizations;
    std::vector <float> candidate_state_speeds;
	std::vector<std::string> candidate_states; // Just to print
	std::vector<std::string>::iterator p_candidate_state;


	// 1. successor_states()
	possible_succesor_states = successor_states();
	for(p_candidate_state = possible_succesor_states.begin();
		p_candidate_state != possible_succesor_states.end() ;
		p_candidate_state++ ){

		std::vector<Vehicle> trajectory;
		float new_cost, inefficiency_cost, goal_distance_cost, lane_distance_cost, penalization, candidate_state_speed;

		// 2. Trajectory for each state
		trajectory = generate_trajectory(*p_candidate_state, predictions);
		if (trajectory.size() == 0) {
			continue;
		}
		succesor_states_trajectories.push_back(trajectory);

		// Cost for each trajectory
		std::tie(new_cost, inefficiency_cost, goal_distance_cost, lane_distance_cost, penalization, candidate_state_speed)
												= calculate_cost(*this, predictions, trajectory);

		costs.push_back(new_cost);
		inefficiency_costs.push_back(inefficiency_cost);
		goal_distance_costs.push_back(goal_distance_cost);
		lane_distance_costs.push_back(lane_distance_cost);
		penalizations.push_back(penalization);
		candidate_states.push_back(*p_candidate_state);
		candidate_state_speeds.push_back(candidate_state_speed);

	}

    std::vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = std::distance(begin(costs), best_cost);

    print_costs(costs, goal_distance_costs, lane_distance_costs, inefficiency_costs, penalizations, candidate_state_speeds, best_idx, candidate_states);

    return succesor_states_trajectories[best_idx];
}

std::vector<std::string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    std::vector<std::string> possible_next_states;
    std::string state;

    state = this->state;
    possible_next_states.push_back("KL");

    bool we_are_not_in_the_rightmost_lane = reference_lane != lanes_available - 1;
    bool we_are_not_in_the_leftmost_lane = reference_lane != 0;

    // During changes of lane we allow only to stay in the current state.
    if(this->get_current_lane() != this->reference_lane){
    	printf("---- CHANGING LANES!!! --------\n");
    	return possible_next_states;
    }

    if(state.compare("KL") == 0) {
        if (we_are_not_in_the_rightmost_lane) {
        	possible_next_states.push_back("PLCR");
        }

        if (we_are_not_in_the_leftmost_lane) {
        	possible_next_states.push_back("PLCL");
        }

    }
    else if (state.compare("PLCR") == 0) {
        if (we_are_not_in_the_rightmost_lane) {
            possible_next_states.push_back("PLCR");
            possible_next_states.push_back("LCR");
        }
        if (we_are_not_in_the_leftmost_lane) {
            possible_next_states.push_back("PLCL");
        }
    }
    else if (state.compare("PLCL") == 0) {
        if (we_are_not_in_the_leftmost_lane) {
            possible_next_states.push_back("PLCL");
            possible_next_states.push_back("LCL");
        }
        if (we_are_not_in_the_rightmost_lane) {
            possible_next_states.push_back("PLCR");
        }
    }
    else if(state.compare("LCL") == 0 || state.compare("LCR") == 0) {

    }

    return possible_next_states;
}

std::vector<Vehicle> Vehicle::generate_trajectory(	std::string state,
													std::map<int, std::vector<Vehicle>> predictions)
{
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */

    std::vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    }
    else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    }
    else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }

    return trajectory;
}

float new_position_constant_acceleration(float x, float v, float a, float t)
{
	return x + v*t + 0.5*t*t*a;
}

float follow_speed(float current_speed, float target_speed, float speed_limit, float max_acceleration, float sampling_time)
{
	float new_speed;
	if(target_speed > current_speed){
		new_speed = current_speed + max_acceleration * sampling_time;
	}
	else{
		new_speed = current_speed - max_acceleration * sampling_time;
	}
	new_speed = SATURATE(new_speed, speed_limit);

	return new_speed;
}

std::vector<float> Vehicle::get_lane_kinematics(std::map<int, std::vector<Vehicle>> predictions,
												int lane) const
{
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    bool printing = false;
    float new_position;
    float new_velocity;
    float new_accel;
    float lane_speed;

    // By default is the maximum speed.
    lane_speed = SPEED_LIMIT_MPH;

     // Car ahead
    Vehicle vehicle_ahead;
    bool there_is_vehicle_ahead = get_vehicle_ahead(predictions, lane, vehicle_ahead);


    if(there_is_vehicle_ahead){
    	// We pay attention to the velocity of the car in front only if it is not too far (100 m)
    	float distance = vehicle_ahead.s - this->s;
    	if(printing) printf("VEHICLE AHEAD: distance: % 4.0f. ", distance);
    	if( distance < DISTANCE_TO_CONSIDER_CARS_IN_LANE ){
    		if(printing) vehicle_ahead.print(" ");
    		lane_speed = std::min((double)SPEED_LIMIT_MPH, vehicle_ahead.velocity-1.0);
    	}
    	else{
    		if(printing) printf("It is too far!!\n");
    	}
    }
    else{
    	if(printing) printf("VEHICLE AHEAD: None detected!!.\n");
    }

    new_velocity = follow_speed(this->velocity, lane_speed, SPEED_LIMIT_MPH, MAX_ACCELERATION_MPH, SAMPLING_TIME);
    new_accel = (new_velocity - this->velocity)*SAMPLING_RATE; //Equation: (v_1 - v_0)/t = acceleration
    new_position = new_position_constant_acceleration(this->s, new_velocity, new_accel, SAMPLING_TIME);


    if(printing) printf("Kinematics for lane %d: Vehicle(s: % 6.1f, v: % 4.1f, lane: %d) -> new(s: % 6.1f, v: % 4.1f)",
    		lane, this->s, this->velocity, this->reference_lane, new_position, new_velocity);
    if(printing) printf(" Speed in lane %d: % 4.1f\n", lane, lane_speed);

    return{new_position, new_velocity, new_accel, lane_speed};

}


std::vector<Vehicle> Vehicle::constant_speed_trajectory()
{
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = position_at(SAMPLING_TIME);
    std::vector<Vehicle> trajectory;

	Vehicle temp_vehicle = Vehicle(*this);
    temp_vehicle.s = next_pos;
    temp_vehicle.velocity= this->velocity;
    temp_vehicle.a = 0;

    trajectory.push_back(Vehicle(*this));
    trajectory.push_back(temp_vehicle);

    return trajectory;
}

std::vector<Vehicle> Vehicle::keep_lane_trajectory(std::map<int, std::vector<Vehicle>> predictions)
{
    /*
    Generate a keep lane trajectory.
    */
    std::vector<Vehicle> trajectory;

    std::vector<float> kinematics = get_lane_kinematics(predictions, this->reference_lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];

	Vehicle temp_vehicle = Vehicle(*this);
    temp_vehicle.s 		  = new_s;
    temp_vehicle.velocity = new_v;
    temp_vehicle.a 		  = new_a;
    temp_vehicle.state = "KL";

    trajectory.push_back(Vehicle(*this));
    trajectory.push_back(temp_vehicle);

    return trajectory;
}

std::vector<Vehicle> Vehicle::prep_lane_change_trajectory(	std::string state,
															std::map<int, std::vector<Vehicle>> predictions)
{
    /*
    Generate a trajectory preparing for a lane change.
    Decide if the speed of the current lane is better than the one on the other lane.
    Adapt to that speed while a spot appears.
    */
    float new_s, new_v, new_a;
    Vehicle vehicle_behind;
    int new_lane;
    std::vector<Vehicle> trajectory;
    std::vector<float> curr_lane_new_kinematics;
    std::vector<float> best_kinematics;
    std::vector<float> next_lane_new_kinematics;


    curr_lane_new_kinematics = get_lane_kinematics(predictions, this->reference_lane);
    new_lane = this->reference_lane + lane_direction[state];

    if (get_vehicle_behind(predictions, this->reference_lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
    }
	else {
        next_lane_new_kinematics = get_lane_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        }
        else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    Vehicle aux_vehicle = Vehicle(*this);
    aux_vehicle.reference_lane = this->reference_lane;
    aux_vehicle.s              = new_s;
    aux_vehicle.velocity       = new_v;
    aux_vehicle.a              = new_a;
    aux_vehicle.state          = state;

    trajectory.push_back(Vehicle(*this));
    trajectory.push_back(aux_vehicle);

    return trajectory;
}

float calculate_safety_distance(float v_mph)
{
    // Meters to have 1 seconds between cars.
    float distance_meters, v_mps;

    v_mps = v_mph * MPH_TO_M_PER_S;
    distance_meters = v_mps * 1.0;

    return distance_meters;
}

std::vector<Vehicle> Vehicle::lane_change_trajectory(
		std::string state,
		std::map<int, std::vector<Vehicle>> predictions)
{
    /*
    Generate a lane change trajectory.
    */
    std::vector<Vehicle> trajectory;
    Vehicle vehicle_to_check;
    int new_lane;
    std::map<int, std::vector<Vehicle>>::iterator it ;
    float safety_distance;

    new_lane = this->reference_lane + lane_direction[state];
    safety_distance = calculate_safety_distance(this->velocity);
    printf("safety_distance %f\n", safety_distance );

    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (it = predictions.begin(); it != predictions.end(); ++it) {
        vehicle_to_check = it->second[0];

        bool vehicles_overlapping = fabs(vehicle_to_check.s - this->s) < (CAR_LENGTH + safety_distance);
        bool is_in_the_new_lane = vehicle_to_check.get_current_lane() == new_lane;

        if ( vehicles_overlapping && is_in_the_new_lane) {
            //If lane change is not possible, return empty trajectory.
        	printf("Change of lane dangerous!!! THERE IS ANOTHER VECHILE IN LANE %d\n", new_lane);
            return trajectory;
        }
    }

    std::vector<float> kinematics = get_lane_kinematics(predictions, new_lane);
    Vehicle aux_vehicle = Vehicle(*this);
    aux_vehicle.reference_lane = new_lane;
    aux_vehicle.s 			   = kinematics[0];
    aux_vehicle.velocity 	   = kinematics[1];
    aux_vehicle.a 		       = kinematics[2];
    aux_vehicle.state          = state;

    trajectory.push_back(Vehicle(*this));
    trajectory.push_back(aux_vehicle);

    return trajectory;
}

void Vehicle::increment(int dt = SAMPLING_TIME)
{
	this->s = position_at(dt);
}

float Vehicle::position_at(float t)
{
    return new_position_constant_acceleration(this->s, this->velocity, this->a, t);
}


bool Vehicle::get_vehicle_behind(std::map<int, std::vector<Vehicle>> predictions,
								 int lane,
								 Vehicle & rVehicle)
{
    /*
    Returns a true if a vehicle is found behind the current vehicle in the given lane, false otherwise.
    The passed reference rVehicle is updated if a vehicle is found.
    */
    int max_s = -10000;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    std::map<int, std::vector<Vehicle>>::iterator it;

    for (it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];

        bool same_lane 				= temp_vehicle.get_current_lane() == lane;
        bool behind_us 				= temp_vehicle.s < this->s;
        bool closest_vehicle_behind	= temp_vehicle.s > max_s;

        if (same_lane && behind_us && closest_vehicle_behind) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

float Vehicle::get_lane_speed(  std::map<int, std::vector<Vehicle>> predictions,
                                int lane) const
{
    float lane_speed = SPEED_LIMIT_MPH;

    Vehicle vehicle_ahead;
    if(get_vehicle_ahead(predictions, lane, vehicle_ahead)){
        float distance = vehicle_ahead.s - this->s;
        if( distance < DISTANCE_TO_CONSIDER_CARS_IN_LANE ){
            lane_speed = std::min((double)SPEED_LIMIT_MPH, vehicle_ahead.velocity-1.0);
        }
    }

    return lane_speed;
}


bool Vehicle::get_vehicle_ahead(std::map<int, std::vector<Vehicle>> predictions,
								int lane,
								Vehicle & rVehicle) const
{
    /*
    Returns a true if a vehicle is found ahead of the current vehicle in the given lane, false otherwise.
	The passed reference rVehicle is updated if a vehicle is found.
    */
    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    std::map<int, std::vector<Vehicle>>::iterator it;

    for (it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];

        bool same_lane 				= temp_vehicle.get_current_lane() == lane;
        // To avoid loosing "sight" of the car when it's close to us, I consider the car length.
        bool ahead_of_us 			= temp_vehicle.s > this->s - CAR_LENGTH;
        bool closest_vehicle_ahead 	= temp_vehicle.s < min_s;

        if (same_lane && ahead_of_us && closest_vehicle_ahead) {
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

std::vector<Vehicle> Vehicle::generate_predictions(int horizon)
{
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
	std::vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {

		float next_s = position_at(i*SAMPLING_TIME);
		float next_v = 0;
		if (i < horizon-1) {
			next_v = (position_at((i+1)*SAMPLING_TIME) - s)/SAMPLING_TIME;
		}

		Vehicle new_vehicle = Vehicle(*this);
		new_vehicle.s = next_s;
		new_vehicle.velocity = next_v;
		new_vehicle.a = 0;

		predictions.push_back(new_vehicle);

  	}

    return predictions;
}

void Vehicle::realize_next_state(std::vector<Vehicle> trajectory)
{
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->reference_lane = next_state.reference_lane;
    this->velocity = next_state.velocity;
    this->a = next_state.a;

}

void Vehicle::configure(std::vector<int> road_data)
{
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
}


