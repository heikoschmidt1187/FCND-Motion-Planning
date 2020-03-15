## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

In planning_utils.py a lot of helper functions for planning are implemented. All the functions have been created in the lesson exercises finally leading to this project. The following table shortly describes meaning of each function:

Item | Description
---|---
Function create_grid | This function creates a grid based on given colliders data. For the project, the data needs to be read from colliders.csv. The no fly zones are determined based on the location and size of the objects from the csv file, an additional safety distance around each object and the object's height in relation to the desired drone flight altitude.
Class Action | A class defining all possible actions for the drone to be performed. This is a base for the planning algorithm decisions. It also contains properties for a cost of the action and the resulting delta movement
Function valid_actions | This operation returns possible movements to be done based on a grind and the current position. Therefor it checks if a movement would lead to a collission with an obstacle or to a "drop off" of the given map if the current position is near the map edges.
Function a_star | As the name states, the function does an A* search for a path from a start position to a goal position. The algorithm is done on a given grid.
Function heuristic | This is the heuristic function needed by A*. In the starter code it's implemented as the euclidean distance between a given position and the goal position.  

The motion_planning.py file contains the flight state machine and the planning algorithm. From the project rubric point the differences between the backyard flyer implementation and the motion planning starter code needs to be described. This is done in the following sections:

The **States** enumeration class has an additional state called *PLANNING*, that indicates the phase where the path to the goal position is planned and the needed waypoints are determinded. Additionally the values for the enum elements are not manually defined anymore but assigned through the usage of the *auto()* call.

The **MotionPlanning** class is the equivalent to the *BackyardFlyer* class and implemented the complete mission for the drone, including all the states to be handled in the flight state machine. Many parts are therefor quite equal to the BackyardFlyer implementation. As with the planning_utils.py file, all the functions are compared in a table:

Function | Equal | Description
---|---|---
init | yes | The initialization of all variables and the registered callback functions are the same. The drone starts in manual mode flight state.
local_position_callback | no | The local position callback handles the takeoff state, the flight maneuvers from waypoint to waypoint and the transition to the landing operation if all waypoints have been visited. The difference between the BackyardFlyer and the MotionPlanning handling is that the motion planner has a dedicated state where the waypoints are determined, so the "calculate_box" call is not present anymore.
velocity_callback | yes | The velocity callback takes care for ending the landing sequence by moving to the disarmed state for drone disarming.
state_callback | no | The state callback function handles the transition between the available flight states. In MotionPlanning the additional *PLANNING* state is handled. After the drone is armed, the *plan_path* function is called to determine the path and waypoints the drone should fly. During this, the planning state is engaged. In the end, the plan_path operation is some kind of equivalent to the calculate_box method in the BackyardFlyer.
arming_transition | no | The arming transition takes care for arming the drone and taking the control from manuall mode to automatic mode. In the MotionPlanning implementation, the call for *set_home_position* is missing as this is done in the plan_path function by reading the position from the colliders.csv file.
takeoff_transition | no | This function handles the initial takeoff of the drone. While in the BackyardFlyer the target altitue is set in the function as a fixed value, in the MotionPlanning it's directly taken from the target position that is set in the plan_path function to a fixed value.
waypoint_transition | yes | The waypoint transition pops waypoint after waypoint from the all_waypoints list and commands the drone to the current waypoint.
landing_transition | yes | This function enters the landing state and commands the drone to fly a landing maneuver.
disarming_transition | yes | This function disarms the drone after landing and returns the control back to manual.
manual_transition | yes | The final function after the mission is accomplished, stops the drone handling and moves back to the manual command mode.
start | yes | The start function to connect the planner drone instance to the simulator
plan_path | no | This function only exists in the MotionPlanning class and is, as already stated, the implementation determining the path and waypoints that the drone should fly. For better readability, the starter code of plan_path is description in the section below.

The **plan_path** function is the heart of the MotionPlanning class as it determines the path to fly from a start position to a goal position. In the starter code, first a target drone altitude and a safety distance to any obstacle is defined. The colliders.csv is read out and a grid is created based on that data, the drone altitude and the safety distance through the utility function *create_grid*. In the starter code the start position is at a defined offset, effectively the center of the grid. The goal is set to be done 10m off from the start position in north and east direction. Then, the *a_star* function is called to get a path from start to goal. The resulting waypoints then are put into the waypoints field is filled with the points from the path. For visualization purposes, they are also sent to the simulator. The waypoints themselves consist of each A* returned point in relation to the start position for north and east, as well as the fixed altitude of the drone.

The *plan_path* function is the main part the path planning algorithm has to be implemented for the next project rubric points, but it doesn't have the be the one and only part. This depends on the actual planning algorithm that should be used.


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.
