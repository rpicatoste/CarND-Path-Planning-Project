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
// In m/s^2 the max is 10
#define MAX_ACCELERATION_METER_S2 (10)
#define MAX_ACCELERATION_MILES_S2 (22.3694)


#define START_LANE (1)

#define WEIGHT_DISTANCE_GOAL (1000.0)
#define WEIGHT_LANE_GOAL	 (1000.0)
#define WEIGHT_EFFICIENCY    (1000.0)

#define LANE_WIDTH (4.0)


#define SAMPLING_TIME (0.02)
#define SAMPLING_RATE (50.0)

#endif /* SRC_MAIN_CONSTANTS_H_ */
