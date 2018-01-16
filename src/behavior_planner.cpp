#include "behavior_planner.h"
#include "sensor_fusion_point.h"
#include <iostream>
#include <vector>
#include <math.h>

#include "spline.h"

constexpr double pi() { return M_PI; }

std::vector<Point> BehaviorPlanner::plan_next_position(	Point &car, 
														std::vector<Point> previous_path, 
														Point end_path,
														std::vector<SensorFusionPoint> sensor_fusion_points,
														std::vector<double> map_waypoints_x,
														std::vector<double> map_waypoints_y,
														std::vector<double> map_waypoints_s)
{

	std::cout << "Entering planner" << std::endl;

    int prev_size = previous_path.size();
    if(prev_size > 0){
    	car.s = end_path.s;
    }

    bool too_close = false;

    // Find ref_v to use
    for(int ii = 0; ii < sensor_fusion_points.size(); ii++){

      // Car is in my car.lane
    	if(sensor_fusion_points[ii].d < (2 + 4.0*car.lane + 2) && sensor_fusion_points[ii].d > (2 + 4*car.lane - 2)){

	        // If using previous points can project s value out 
	        sensor_fusion_points[ii].s += ((double)prev_size*0.02*sensor_fusion_points[ii].speed);
	        // Check s values greater than mine and s gap
	        if((sensor_fusion_points[ii].s > car.s) && ((sensor_fusion_points[ii].s - car.s) < 30)){
	          
	        	too_close = true;
	        	if(car.lane > 0){
	            	car.lane = 0;
	        	}
        	}
    	}
    }


    if(too_close){
    	car.ref_vel -= 0.224;
    }
    else if(car.ref_vel < 49.5){
    	car.ref_vel += 0.224;
    }


  	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
    // Create very spaced (30m) waypoints, then interpolate with spline.
    std::vector<Point> pts;


    Point ref = Point();

    if(prev_size < 2){
		// Use 2 points that make the path tangent to the car.
		ref.x = car.x;
	    ref.y = car.y;
	    ref.yaw_rad = car.yaw_rad;
	    ref.yaw_deg = car.yaw_deg;

		Point prev_car = Point();
		prev_car.x = car.x - cos(car.yaw_rad);
		prev_car.y = car.y - sin(car.yaw_rad);              

		pts.push_back(prev_car);
		pts.push_back(car);              

    }
    // use the previous path's end point as starting reference
    else{
		// Redefine
		ref.x = previous_path[prev_size-1].x;
		ref.y = previous_path[prev_size-1].y;

		Point ref_prev = Point();
		ref_prev.x = previous_path[prev_size-2].x;
		ref_prev.y = previous_path[prev_size-2].y;

		ref.yaw_rad = atan2(ref.y - ref_prev.y, ref.x - ref_prev.x );
		ref.yaw_deg = ref.yaw_rad * 180.0 / M_PI;

		// Use 2 points that make the path tangent to the previous path's end point
		pts.push_back(ref_prev);
		pts.push_back(ref);              

    }
    //ref.print("Ref: ");

    Point next_wp0 = Point( getXY(car.s + 30,
                                  (2+4*car.lane),
                                  map_waypoints_s,
                                  map_waypoints_x,
                                  map_waypoints_y) ); 
    Point next_wp1 = Point( getXY(car.s + 60,
                                  (2+4*car.lane),
                                  map_waypoints_s,
                                  map_waypoints_x,
                                  map_waypoints_y) );
    Point next_wp2 = Point( getXY(car.s + 90,
                                  (2+4*car.lane),
                                  map_waypoints_s,
                                  map_waypoints_x,
                                  map_waypoints_y) );

    pts.push_back(next_wp0);            
    pts.push_back(next_wp1);
    pts.push_back(next_wp2);


    for(int ii = 0; ii < pts.size(); ii++){
		Point shift = Point();
		shift = pts[ii] - ref;
		pts[ii] = shift.rotate_by_angle(-ref.yaw_rad);
    }

    // Create a spline
    tk:: spline my_spline;

    // Set (x,y) points to the spline
    my_spline.set_points( Point::get_vector_x_from_list(pts), 
                          Point::get_vector_y_from_list(pts));

    // Define the actual (x,y) points we will use for the planner
    std::vector<Point> next_vals;
    
    // Start with all of the previous path points from last time
    for (int ii = 0; ii < previous_path.size(); ii++){
		Point new_point = Point(previous_path[ii].x, previous_path[ii].y);
		next_vals.push_back(new_point);
	}

    // Calculate hot to break up spline points so that we travel at our desired reference velocity
    Point target = Point();
    target.x = 30.0;
    target.y = my_spline(target.x);

    double x_add_on = 0;

    // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
    for( int ii = 1; ii <= 50-previous_path.size(); ii++ ){
		double N = (target.module()/(0.02*car.ref_vel/2.24)); // div by 2.24 for MPH

		Point point = Point();
		point.x = x_add_on + (target.x)/N;
		point.y = my_spline(point.x);

		x_add_on = point.x;

		// Rotates back to normal after rotating it earlier
		point = point.rotate_by_angle(ref.yaw_rad);
		point = point + ref;

		next_vals.push_back(point);

    }

	return next_vals;
}



// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> BehaviorPlanner::getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


