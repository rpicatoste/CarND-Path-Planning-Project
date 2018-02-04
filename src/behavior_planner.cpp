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
															std::vector<SensorFusionPoint> other_cars_raw)
{
	printf("\n----- Entering planner. State: %s. Lane: (reference: %d real: %d). s: % 4.0f m. d: %3.1f. Speed: % 3.0f mph. Number of vehicles: %d -----\n\n",
	        car.state.c_str(), car.reference_lane, car.get_current_lane(), car.s, car.d, car.velocity, other_cars_raw.size());

    int prev_size = previous_path.size();
    if(prev_size > 0){
    	car.s = end_path.s;
    }

     // Convert the SensorFusionPoints to a Vehicle list.
    std::vector<Vehicle> other_cars;
    for(int ii = 0; ii < other_cars_raw.size() ; ++ii){
    	other_cars.push_back(Vehicle(other_cars_raw[ii], *this));
    }

	// For each vehicle, generate the prediction of the path and store in predictions.
    std::map<int ,std::vector<Vehicle> > all_predictions;

    for(int ii = 0; ii < other_cars.size() ; ++ii)
    {
		all_predictions[ii] = other_cars[ii].generate_predictions();

        // Print
        std::string pred_text = "";
        for(int jj = 0; jj<all_predictions[ii].size(); ++jj){
        	pred_text += "| Pred " + std::to_string(jj) + " " + all_predictions[ii][jj].relative_position_to_string(car) + ")";
        }

        // Do not delete this print. Useful understanding the other cars and their predictions.
        //printf("  Car %02d. Preds: %s\n", ii, pred_text.c_str() );
    }

    // Choose next state
	std::vector<Vehicle> trajectory = car.choose_next_state(all_predictions);
    car.realize_next_state(trajectory);

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

		// Do not delete, instersting when working on the points passed to the simulation.
		//next_waypoints_raw[ii].print("New waypoints relative: ");
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


