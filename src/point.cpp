#include <iostream>
#include <math.h>
#include <functional>

#include "behavior_planner.h"
#include "cost.h"
#include "helpers.h"
#include "main_constants.h"
#include "point.h"


float REACH_GOAL = pow(10, 6);
float EFFICIENCY = pow(10, 5);

extern std::vector<double> *global_p_map_waypoints_x;
extern std::vector<double> *global_p_map_waypoints_y;
extern std::vector<double> *global_p_map_waypoints_s;

Point::Point()
{
	this->ref_vel = 0.0;
}

Point::Point(std::vector<double> xy)
{
	this->x = xy[0];
	this->y = xy[1];

	this->ref_vel = 0.0;
}

Point::Point(double x, double y)
{
	this->x = x;
	this->y = y;

	this->ref_vel = 0.0;
}

Point::Point(double x, double y, double s, double d)
{
	this->x  = x;
	this->y  = y;
	this->s  = s;
	this->d  = d;

	this->ref_vel = 0.0;
}

Point::Point(double x, double y, double s, double d, double yaw_deg, double speed)
{
	this->x  = x;
	this->y  = y;
	this->s  = s;
	this->d  = d;
	this->yaw_deg = yaw_deg;
	this->yaw_rad = deg2rad(yaw_deg);

	this->v = speed;

	this->ref_vel = 0.0;
}

Point::Point(SensorFusionPoint other_vehicle)
{
    BehaviorPlanner bp = BehaviorPlanner();
    auto xy = bp.getXY(other_vehicle.s,
						other_vehicle.d,
						*global_p_map_waypoints_s,
						*global_p_map_waypoints_x,
						*global_p_map_waypoints_y);
    this->x = xy[0];
    this->y = xy[1];
	this->s  = other_vehicle.s;
	this->d  = other_vehicle.d;
	this->yaw_deg = 0.0;
	this->yaw_rad = deg2rad(yaw_deg);

	this->v = sqrt(other_vehicle.vx*other_vehicle.vx + other_vehicle.vy*other_vehicle.vy);

	this->ref_vel = 0.0;
	this->lane = 0;
}

Point::Point(int lane, float s, float v, float a, std::string state)
{
	this->lane = lane;
	this->s = s;
	this->v = v;
	this->a = a;
	this->state = state;


}

Point::~Point() 
{

}


void Point::print(std::string text)
{
	std::cout << text;
	std::cout << "Point (x: " << this->x << ", y: " << this->y << ", s: " << this->s << ", d: " << this->d  << ", yaw_deg: " << this->yaw_deg << ")." << std::endl;

}


double Point::module(void)
{
	Point zero = Point(0.0, 0.0);

	return this->distance(zero);
}

double Point::distance(Point p)
{
	return distance(*this, p);
}


double Point::distance(Point p1, Point p2)
{
	return sqrt( (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) );
}

Point Point::rotate_by_angle(double angle)
{
	Point rotated = Point(*this);

	rotated.x = (this->x * cos(angle) - this->y * sin(angle));
	rotated.y = (this->x * sin(angle) + this->y * cos(angle));

	return rotated;
}

std::string Point::to_string(void)
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


std::string Point::relative_to_string(const Point& other_point)
{
    char aux[10] = "";
	std::string txt = "(";
	const char* format = "%5.0f";

    sprintf(aux, format, this->s - other_point.s);
    txt += std::string(", s: ") + std::string(aux);

    sprintf(aux, format, this->d - other_point.d);
    txt += std::string(", d: ") + std::string(aux);

	txt += std::string(")");

	return txt;
}



std::ostream& operator<<(std::ostream& os, const Point& point)
{

    return os << "(x:" << point.x << ", y:" << point.y << "// s:" << point.s << ", d:" << point.d << ")";
}


std::vector<Point> Point::choose_next_state(std::map<int, std::vector<Point>> predictions)
{

	// 1. successor_states()
	std::vector<std::string> possible_succesor_states = successor_states();

	std::vector<std::vector<Point>> succesor_states_trajectories;
	std::vector <float> costs;

	for(	std::vector<std::string>::iterator p_candidate_state = possible_succesor_states.begin();
			p_candidate_state != possible_succesor_states.end() ;
			p_candidate_state++ ){

		// 2. generate_trajectory(string state, map<int, vector<Vehicle>> predictions)
		std::vector<Point> trajectory = generate_trajectory(*p_candidate_state, predictions);
		succesor_states_trajectories.push_back(trajectory);

		// 3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory)
		float new_cost;
		new_cost = calculate_cost(*this, predictions, trajectory);
		costs.push_back(new_cost);
	}

	//std::vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
	int best_idx=0;//int best_idx = distance(begin(costs), best_cost);
    return succesor_states_trajectories[best_idx];
}




std::vector<Point> Point::generate_trajectory(std::string state, std::map<int, std::vector<Point>> predictions)
{
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */

	std::vector<Point> trajectory;
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



std::vector<Point> Point::prep_lane_change_trajectory(std::string state, std::map<int, std::vector<Point>> predictions)
{
    /*
    Generate a trajectory preparing for a lane change.
    */
    float new_s;
    float new_v;
    float new_a;
    Point vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    std::vector<Point> trajectory = {Point(this->lane, this->s, this->v, this->a, this->state)};
    std::vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];

    }
	else {
		std::vector<float> best_kinematics;
		std::vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
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

    trajectory.push_back(Point(this->lane, new_s, new_v, new_a, state));
    return trajectory;
}

std::vector<Point> Point::lane_change_trajectory(std::string state, std::map<int, std::vector<Point>> predictions)
{
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    std::vector<Point> trajectory;
    Point next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (std::map<int, std::vector<Point>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Point(this->lane, this->s, this->v, this->a, this->state));
    std::vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Point(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}


bool Point::get_vehicle_behind(std::map<int, std::vector<Point>> predictions, int lane, Point & rVehicle)
{
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -1;
    bool found_vehicle = false;
    Point temp_vehicle;
    for (std::map<int, std::vector<Point>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}


bool Point::get_vehicle_ahead(std::map<int, std::vector<Point>> predictions, int lane, Point & rVehicle)
{
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = 4500.0;
    bool found_vehicle = false;
    Point temp_vehicle;
    for (std::map<int, std::vector<Point>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (	temp_vehicle.lane == this->lane &&		// Same lane
        		temp_vehicle.s > this->s && 			// Ahead of us
				temp_vehicle.s < min_s) {				// The closest vehicle ahead of us
            min_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}


std::vector<float> Point::get_kinematics(std::map<int, std::vector<Point>> predictions, int lane)
{
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    float max_velocity_accel_limit = MAX_ACCELERATION + this->v;
    float new_position;
    float new_velocity;
    float new_accel;
    Point vehicle_ahead;
    Point vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        }
        else {
            float max_velocity_in_front =  (vehicle_ahead.s - this->s - this->preferred_buffer)
            							  + vehicle_ahead.v - 0.5 * (this->a);

			new_velocity = std::min(	std::min(max_velocity_in_front, max_velocity_accel_limit),
										this->ref_vel );
        }
    }
    else {
        new_velocity = std::min(max_velocity_accel_limit, this->ref_vel);
    }

    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    return{new_position, new_velocity, new_accel};

}



std::vector<Point> Point::keep_lane_trajectory(std::map<int, std::vector<Point>> predictions)
{
    /*
    Generate a keep lane trajectory.
    */
    std::vector<Point> trajectory = {Point(lane, this->s, this->v, this->a, state)};
    std::vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Point(this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}


std::vector<Point> Point::constant_speed_trajectory()
{
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = position_at(1);
    std::vector<Point> trajectory = {Point(this->lane, this->s, this->v, this->a, this->state),
    								 Point(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}


float Point::position_at(int t)
{
    return this->s + this->v*t + this->a*t*t/2.0;
}

std::vector<std::string> Point::successor_states()
{
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
	std::vector<std::string> states;
    states.push_back("KL");
    std::string state = this->state;

    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0) {
        if (lane != LANES_AVAILABLE - 1) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0) {
        if (lane != 0) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

