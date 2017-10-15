# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## Implemenation

### Designing the path - general approach
My approach is based on the usage of [spline](http://kluge.in-chemnitz.de/opensource/spline/) function which given some points try to fit a fairly smooth spline. After having defined my spline I provide more points between the start and the end of the curve controlling my car's speed.

### Working with coordinates
In this project I mainly work with Frenet coordinates s&d. In frenet, "s" is the distance along a lane while "d" is the distance away from the center dividing line of the road. Conversion between cartesian coordinates and frenet coordinates is conducted in this project a lot. Working with Frenet coordinates makes calculations easier and it is more human firendly.

### Getting started with the data
The simulator runs a cycle every 20 ms (50 frames per second), but my C++ path planning program will provide a new path at least one 20 ms cycle behind. The simulator will simply keep progressing down its last given path while it waits for a new generated path. My path is a vector of 50 elements containing x and y coordinates of my car. If the simulator didn't have the time to process all the points in the path then I append the new path points to the existing ones. This approach is actually a queue (FIFO). This way I ensure that every lane change will be smooth and most important in combination with spline approach will ensure that I will satisfy the criteria of speed, accelaration and jerk. In main.cpp (lines 257 - 261) I check whether there are path points pending and I consider my current car position the last point in the path. These pending points will be the first to be pushed back to the new path plan (lines 386-388 in `main.cpp`). Additionally, I calculate the x, y, yaw and velocity based off the end values of the remaining path, in order to have the behavior planner begin from the end of the old path.

### Path planning 
Next the function *laneIndicator* in `behaviorPlanner.cpp` (line 68) takes decision on what lane shall the car continue. The class *BehaviorPlanner* contains my path planner, the cost functions and generally most of the logic. Before choosing to which lane my car shall continue I firstly identify the lane I am currently in with the *identifyLane* function (line 11). Then I calculate the cost of every lane with the *laneCostEstimator* function(line 108). If the function suggested me a new lane I check whether it is safe or not with the *isLaneSafe* function (line 165). After the lane has been checked for safety I set my target speed (*adjustVehicleTargetSpeed* line 91). Now that I know to which lane my car shall continue it is time to make my waypoints for the spline function. To draw the spline we will need starting and ending points. I use my previous car path data as a starting point (lines 313-342 in `main.cpp`) and as the end point I create evenly 30 m spaced points ahead of the starting reference (lines 352-362). In addition I shift car reference angles to 0 degrees for easier calculations (lines 366-372) and before sending to simulator I rotate them back (lines 421- 426). According to the desired speed I calculate the points within the spline untill i reach a stack of 50 including the previous values and then i send them to the simulator.

### Cost function

The cost function *laneCostEstimator* is located in `behaviorPlanner.cpp` (line 108). It is responsible to evaluate each lane and decide which one will follow. It takes account of the distance from the front vehicle, the back vehicle or no vehicle at all in case of a free lane. It does not permit changing lanes that have more than one distance to avoid lateral accelarion and jerk. It also takes account of the previous costs of the lane to avoid spurious and continuous change lane behaviors. Finally it slightly benefits the current lane especially in case of free road.

### Safe transition

The safe transition between lanes is being handled by the *isLaneSafe* function in `behaviorPlanner.cpp` (line 165). A very general check is being made whereas any car in front or back exists within a range of 10 m (frenet) both in the car's current and target lane. If this criteria is not true then a projection of my car's position and the vehicles' in the  target lane is being made. If the distance predicted is more than 10 m (frenet) then the transition is considered safe.

### Speed of the car

The speed of the car can be controlled by the path points themselves. Every 20 ms the car moves to the next waypoint so the velocity will be (Distance from previous waypoint/ 20 ms). Given the fact that I know my target speed *ref\_vel* I can devide the desired distance *target\_dist* I want to travel (in my case 30m) by N equally spaced intervals by the formula: *N = (target\_dist/(0.02\*ref\_vel))* (line 412 in `main.cpp`). Now if the car has a lower speed than the target speed for every path point it acelarates by 0.224 mph. If its speed is higher than the target speed it deaccelerates by 0.224 mph (lines 404- 414 in `main.cpp`). This number satisfies the  limitation of longitudinal acceleration and jerk. The target speed is defined by a simple logic. The car shall always try to reach the maximum speed limit of 50 mph (in my case 49.5 mph). The only reason to slower speed it to be blocked by another car in front so it adjusts its speed to the front car's speed. Since the coordinate conversion functions between cartesian and frenet coordinates provided do not export accurate results, an oscilltion in speed is introduced when following a car. This issue has been resolved in `behaviorPlanner.cpp` (lines 98-104) by checking also the future speed of the car and not only the frenet coordinate s. If the car in front deaccelerates very aggressively our car will do its best to slower speed by 0.224 mph every 20 ms.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

