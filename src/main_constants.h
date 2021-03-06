/*
 * main_constants.h
 *
 *  Created on: 24 Jan 2018
 *      Author: Pica
 */

#ifndef SRC_MAIN_CONSTANTS_H_
#define SRC_MAIN_CONSTANTS_H_


#define LANES_AVAILABLE (3)


#define	SPEED_LIMIT_MPH (49)
#define NUM_LANES (3)
#define	GOAL_S (6945)
#define	GOAL_LANE (0)

// In m/s^2 the max is 10 (22.37 mphs). With margin 9.5m/s2 (21.25)/8.5(19.0)
#define MAX_ACCELERATION_METER_S2 (8.5)
#define MAX_ACCELERATION_MPH (19.0)


#define START_LANE (1)

#define WEIGHT_DISTANCE_GOAL (1000.0)
#define WEIGHT_LANE_GOAL	 (1000.0)
#define WEIGHT_EFFICIENCY    (1000.0)
#define LANE_CHANGE_PENALIZATION (200.0)
#define LANE_CHANGE_BONIFICATION (-200.0)

#define LANE_WIDTH (4.0)


#define SAMPLING_RATE (50.0)
#define SAMPLING_TIME (0.02)
#define SAMPLING_TIME_2 (0.0004)

#define M_PER_S_TO_MPH (2.24)
#define MPH_TO_M_PER_S (0.446)

#define CAR_LENGTH (5.0)

#define DISTANCE_TO_START_GOING_TO_GOAL_LANE (200.0)
#define DISTANCE_TO_CONSIDER_CARS_IN_LANE (25.0)



#define SATURATE(x,max) (x > max) ? max : ((x < -max) ? -max : x)

#endif /* SRC_MAIN_CONSTANTS_H_ */
