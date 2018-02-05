# Self-Driving Car Nanodegree Program - Term 3
## Project 1 - Path Planning
**Ricardo Picatoste**

This is the Path planning project, where we write a program that will take the control of the car. 
The information from the sensors will be given already processed with the position and speed of the surrounding cars, and the goal will be to navigate the road until the final destination is reached.

I recorded a video with the resulting software after some tuning of the parameters, that can be found here:

[![Project Video](https://img.youtube.com/vi/-uRjtP7aQNs/0.jpg)](https://youtu.be/-uRjtP7aQNs)

The car does several changes of lane. 
At the end, when it is close to the goal 4.23 miles, it can be appreciated how it tends to avoid changing lane, as since the goal is approaching, it will just try to get to it.

Once the point of the 4.23 miles is passed, however, the car starts "running" again and overtakes the car in front.

This is due to the fact that near the goal, going faster has diminishing importance as compared to stay in the proper lane.


### Running the code:

I used this code in Windows 10. 
To compile and run it, I use the Bash no Ubuntu on Windows.
To do it, these are the steps:
* Clone the project 
* Go to the folder: */project/build*
* To compile the project: *cmake .. && make *
* And to run: *./path_planning*
* In parallel, run the simulator in Windows. 


### Path planning

The path planning software is based on a finite state machine. 
To start, I used the video of the walkthrough, which helped making sense out of the json messages interchanged with the simulator.

In addition, I used as well as starting point the code resulting from the different exercises done in the path planing chapter.

This was a starting point but the code changed heavily from there on. 

The most important part of the program in order to decide what action to take are the generation of trajectories and the cost functions.

### Succesor states and trajectory generation.

In this function the trajectory which will correspond to each of the possible actions (or next possible states) is computed.

The possible successor states depend of the position of the car in the road.
If we are in the side lanes, the options will be to keep lane or to change to the lane opposed to the limit of the road.

For the states that carry out a lane change, it will be checked if there is a car in the goal lane, including a safety distance. 
Otherwise, a lane change will not be allowed.

In order to not get blocked waiting for a spot in the goal lane, if the other side of the lane change becomes better in terms of cost, the change will be done in the other directions.


### Cost functions

The decission of the action to take is done by the cost functions, will will be added in a final cost. 
The main costs are the following.

The new_inefficiency_cost will try to penalize slow speeds. 
This cost will be higher when we are further from our goal lane and s, because there is time to move to he final lane destination if we are far, and going faster will be more important in this situation.
For this reason the multiplier for this cost will reward more with the distance to the goal, but will a limit, in order not to shadow all the other costs with big distances.

The new_goal_distance_cost will reward the trajectories that bring us towards the goal. 
In this setup this cost will not have a big effect, but in general is good to have it, as it will push the car towards the goal.

The new_goal_lane_cost will reward being in the goal lane, but its effect will be only important when we are near the final goal. 
When the car is far from the goal, it will not matter in which lane we are, as we will have time to reach it.

There will be as well a change_lane_penalization, to reward not changing lane when the improvement (in speed, for example) is too small.
There will also be a change_lane_bonification, but this will only be active when a lane change has been decided. 
The purpose of the latter is only to let the finite state machine pass from prepare lane change to actually execute it.


### Others

The lane_speed will be by defaul the maximum speed. 
Then the cars around us will be checked, and for each lane, those cars in front of us (higher s value) will be considered.
If one of those cars is close enough to our car (in the s axis), its speed will become the lane_speed, as we cannot go faster than the car in front of us.

### Structure of the code:

The different .cpp and .h and their content:
* main: Start the program and interchange data with the simulator.
* behavior_planner: The planner that will call the different functions deciding the actions to take, and will generate the list of points to send to the simulator to follow.
* vehicle: Class defining a vehicle, both for our car and for the other cars.
* cost: Cost functions definition.
* sensor_fusion_point: Structure for the sensor fusion data received from the simulator.
* helpers: Some helper functions.




