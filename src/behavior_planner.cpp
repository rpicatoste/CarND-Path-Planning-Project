#include "behavior_planner.h"

#include <iostream>
#include <map>
#include <math.h>
#include <vector>

#include "main_constants.h"
#include "sensor_fusion_point.h"
#include "spline.h"
#include "vehicle.h"

constexpr double pi() { return M_PI; }

std::vector<Vehicle> BehaviorPlanner::plan_next_position(	Vehicle &car, 
															std::vector<Vehicle> previous_path,
															Vehicle end_path,
															std::vector<SensorFusionPoint> other_cars)
{
	printf("\n----------- Entering planner. Current reference lane: %d. Speed: % 3.0f Number of vehicles: %d -----------\n\n",
			car.reference_lane, car.velocity, other_cars.size());

    int prev_size = previous_path.size();
    if(prev_size > 0){
    	car.s = end_path.s;
    }

    // ********************************************************************************************************************
    // Porting the FSM

	// For each vehicle, generate the prediction of the path and store in predictions.
    std::map<int ,std::vector<Vehicle> > all_predictions;
    std::vector<SensorFusionPoint>::iterator p_vehicle;
    int vehicle_counter = 0;


    p_vehicle = other_cars.begin();
    vehicle_counter = 0;
    while(p_vehicle != other_cars.end())
    {
		int vehicle_id;
		std::vector<SensorFusionPoint> predictions_for_current_vehicle_raw;
        std::vector<Vehicle> predictions_for_current_vehicle;


		vehicle_id = vehicle_counter;
        predictions_for_current_vehicle_raw = p_vehicle->generate_predictions();
        for(int ii = 0; ii < predictions_for_current_vehicle_raw.size(); ii++){
        	predictions_for_current_vehicle.push_back(Vehicle(predictions_for_current_vehicle_raw[ii], *this));
        }
        all_predictions[vehicle_id] = predictions_for_current_vehicle;
        p_vehicle++;
        vehicle_counter++;

        // Print
        std::string pred_text = "";
        for(int jj = 0; jj<predictions_for_current_vehicle.size(); jj++){
        	pred_text += "(" + predictions_for_current_vehicle[jj].relative_position_to_string(car) + ")";
        }
        printf("  Car %02d. Preds: %s\n", vehicle_id, pred_text.c_str() );
    }

    // Choose next state
	//vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
    std::vector<Vehicle> trajectory = car.choose_next_state(all_predictions);
    car.realize_next_state(trajectory);

	// 1. successor_states()
    //vector<string> possible_succesor_states = successor_states();

    printf("(Before) Car desired lane: %d, desired speed: %f\n", car.reference_lane, car.velocity);
// JUST DECIDE LANE AND REF_VEL. THEY WILL DEPEND ON STATE, AND StATE ON THEM...

    // ********************************************************************************************************************
 /*   bool too_close = false, car_in_my_lane = false;

    // Find ref_v to use
    for(int ii = 0; ii < other_cars.size(); ii++){

    	// Car is in my car.lane
    	car_in_my_lane = other_cars[ii].d < (car.get_desired_s() + 2) && other_cars[ii].d > (car.get_desired_s() - 2);

    	if(car_in_my_lane){
	        // If using previous points can project s value out 
    		other_cars[ii].s += ((double)prev_size*0.02*other_cars[ii].speed);
	        // Check s values greater than mine and s gap
	        if((other_cars[ii].s > car.s) && ((other_cars[ii].s - car.s) < 30)){
	          
	        	too_close = true;
	        	if(car.reference_lane > 0){
	            	car.reference_lane = 0;
	        	}
        	}
    	}
    }

    if(too_close){
    	car.velocity -= 0.224;
    }
    else if(car.velocity < SPEED_LIMIT_MPH-0.5){
    	car.velocity += max_delta_v;
    }

    printf("(After ) Car desired lane: %d, desired speed: %f\n", car.reference_lane, car.velocity);
*/
    // TODO if we don't give an initial speed, this crashed in the spline generation.
 /*   float max_delta_v = MAX_ACCELERATION_MILES_S2*SAMPLING_TIME;
    if(car.velocity < SPEED_LIMIT_MPH){
    //if(car.s < 130.0){
    	car.velocity += max_delta_v;
    }
    printf("Car s: %f\n", car.s);*/
    // ********************************************************************************************************************

  	// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    // Create very spaced waypoints, then interpolate with spline.
    std::vector<Vehicle> next_waypoints_raw;
    Vehicle reference_point = Vehicle();

    if(prev_size < 2){
		// Use 2 points that make the path tangent to the car.
		reference_point.x = car.x;
	    reference_point.y = car.y;
	    reference_point.yaw_rad = car.yaw_rad;
	    reference_point.yaw_deg = car.yaw_deg;

		Vehicle prev_car = Vehicle();
		prev_car.x = car.x - cos(car.yaw_rad);
		prev_car.y = car.y - sin(car.yaw_rad);              

		next_waypoints_raw.push_back(prev_car);
		next_waypoints_raw.push_back(car);              

    }
    // Use the previous path's end point as starting reference
    else{
    	reference_point.x = previous_path[prev_size-1].x;
		reference_point.y = previous_path[prev_size-1].y;

		Vehicle ref_prev = Vehicle();
		ref_prev.x = previous_path[prev_size-2].x;
		ref_prev.y = previous_path[prev_size-2].y;

		reference_point.yaw_rad = atan2(reference_point.y - ref_prev.y, reference_point.x - ref_prev.x );
		reference_point.yaw_deg = reference_point.yaw_rad * 180.0 / M_PI;

		// Use 2 points that make the path tangent to the previous path's end point
		next_waypoints_raw.push_back(ref_prev);
		next_waypoints_raw.push_back(reference_point);
    }


    // At 50 mph, the distance_to_next_waypoint is 30.0 meters.
    float distance_to_next_waypoint = car.velocity * 3.0/5.0;
    const float min_distance_to_next_waypoint = 3.0;
    distance_to_next_waypoint = (distance_to_next_waypoint > min_distance_to_next_waypoint) ?
    														distance_to_next_waypoint : min_distance_to_next_waypoint;

    Vehicle next_wp0 = Vehicle( getXY(car.s +   distance_to_next_waypoint, (2+4*car.reference_lane)) );
    Vehicle next_wp1 = Vehicle( getXY(car.s + 2*distance_to_next_waypoint, (2+4*car.reference_lane)) );
    Vehicle next_wp2 = Vehicle( getXY(car.s + 3*distance_to_next_waypoint, (2+4*car.reference_lane)) );

    next_waypoints_raw.push_back(next_wp0);            
    next_waypoints_raw.push_back(next_wp1);
    next_waypoints_raw.push_back(next_wp2);

    // Make waypoints relative to reference_point.
    for(int ii = 0; ii < next_waypoints_raw.size(); ii++){
		Vehicle shift = Vehicle();

		shift = next_waypoints_raw[ii] - reference_point;
		next_waypoints_raw[ii] = shift.rotate_by_angle(-reference_point.yaw_rad);

		next_waypoints_raw[ii].print("New waypoints relative: ");
    }

    std::vector<Vehicle> next_waypoints_for_the_simulator;
    next_waypoints_for_the_simulator = convert_raw_waypoints_to_simulator_waypoints(
    		car,
			reference_point,
    		next_waypoints_raw,
    		previous_path);

	return next_waypoints_for_the_simulator;
}





std::vector<Vehicle> BehaviorPlanner::convert_raw_waypoints_to_simulator_waypoints(
		Vehicle &car,
		Vehicle reference_point,
		std::vector<Vehicle> next_waypoints_raw,
		std::vector<Vehicle> previous_path)
{
    // Create a spline
    tk:: spline my_spline;

    // Set (x,y) points to the spline
    my_spline.set_points( Vehicle::get_vector_x_from_list(next_waypoints_raw), 
                          Vehicle::get_vector_y_from_list(next_waypoints_raw));

    // Define the actual (x,y) points we will use for the planner
    std::vector<Vehicle> next_waypoints_for_the_simulator;

    // Start with all of the previous path points from last time
    for (int ii = 0; ii < previous_path.size(); ii++){
		Vehicle new_point = Vehicle(previous_path[ii].x, previous_path[ii].y);
		next_waypoints_for_the_simulator.push_back(new_point);
	}

    // Calculate hot to break up spline points so that we travel at our desired reference velocity
    Vehicle target = Vehicle();
    target.x = 30.0;
    target.y = my_spline(target.x);

    double x_add_on = 0;
    // Fill up the rest of our path planner after filling it with previous points, here we will
    // always output 50 points
    for( int ii = 1; ii <= 50-previous_path.size(); ii++ ){
		double N = (target.module()/(0.02*car.velocity/2.24)); // div by 2.24 for MPH

		Vehicle point = Vehicle();
		point.x = x_add_on + (target.x)/N;
		point.y = my_spline(point.x);

		x_add_on = point.x;

		// Rotates back to normal after rotating it earlier
		point = point.rotate_by_angle(reference_point.yaw_rad);
		point = point + reference_point;

		next_waypoints_for_the_simulator.push_back(point);
    }

    return next_waypoints_for_the_simulator;
}


// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> BehaviorPlanner::getXY(	double s, double d)
{
	int prev_wp = -1;

	while(s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%map_waypoints_x.size();

	double heading = atan2((map_waypoints_y[wp2]-map_waypoints_y[prev_wp]),(map_waypoints_x[wp2]-map_waypoints_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-map_waypoints_s[prev_wp]);

	double seg_x = map_waypoints_x[prev_wp]+seg_s*cos(heading);
	double seg_y = map_waypoints_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


