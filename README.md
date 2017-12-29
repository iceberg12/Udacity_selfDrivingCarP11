# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab] (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

| CRITERIA  | MEETS SPECIFICATIONS |
| ------------- | ------------- |
| The car is able to drive at least 4.32 miles without incident.  | The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail. |
| The car drives according to the speed limit. | The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.  |
| Max Acceleration and Jerk are not Exceeded.  | The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.  |
| Car does not have collisions. | The car must not come into contact with any of the other cars on the road.  |
| The car stays in its lane, except for the time between changing lanes.  | The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.  |
| The car is able to change lanes  | The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.  |

### Input

Here is the data provided from the Simulator to the C++ Program.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

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

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates]. 

#### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 10 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Algorithm and Implementation

[trajectory]: ./images/trajectory_generation.jpeg
[state]: ./images/state_optimization.jpeg
[frenet]: ./images/Frenet.png
[behavior]: ./images/behavior.png

The flow is summarized into two stages: state selection and trajectory generation.

#### Trajectory Generation

From Bellman's principle of optimality, at every sampling step we can re-use the proposed trajectory from the previous steps and simply add a few more optimal steps to maintain a fixed horizon of trajectory (in this case, 50 time steps of 20ms each). This will minimize any abrupt change due to previous optimal trajectory and current optimal trajectory and reduce unnecessary jerks.

![alt text][frenet]

Assume we have n planned steps which our car hasn't consume from the previous sampling step. Our task is finding the next 50 - n steps that suits the situation. This is first done create a Spline smoothed curve through two points s at the end of the previous planned trajectory and three points far ahead which Frenet coordinates are determined by future (s distance, d distance) (s = [30, 60, 90] m away from the trajectory end, d is the distance from road center and d directly indicates the goal lane). Certainly, the spline calculation can be made easy by advancing the current car coordinate to the end of the previous path. After having the spline, we can use the current car speed to generate 50 - n points along this spline. These points should have consecutive short distance so our car can move between them within 0.02 s (our sampling time).

![alt text][trajectory]

#### Finding Goal Lane

Before constructing the above trajectory, one variable we need to determine is the lane we want the car to go. In principle, this involves predicting where other cars go next and how our car should behave.

![alt text][behavior]

Predicting other car behavior can be done using their current position and speed. To plan our car behavior, a few points are considered:

1. When to change lane. This includes close vehicle ahead detection and whether the other lanes are safe to move to. Here we need to predict whether our car will collide with others in a few second ahead.
2. Reduce speed when the vehicle ahead is slow. Also speed up to the speed limit (50 mph or 22.35 m/s) when there is no close car ahead.
3. Consider which lane to change to. The must condition is that other lanes are safe. If we are in the middle lane 1 and both lane 0 and 2 (left and right) are safe, I used a simple cost function equal to the sum of (1) distance from closest ahead car if I move to that lane and (2) velocity of that car. This is to avoid moving into a busy lane that might force us to switch lane again.
4. During lane changing, we shouldn't plan to change again so soon. Just stabilize the car into the goal lane first, before considering lane change again.
5. I added a discomfort_tailing distance, which keep increasing if there is a slow vehicle ahead but all other potential lanes are not safe; it resets when the car changes lane, or auto-reset from time to time. This encourage our car to slow down further, away from the car ahead, to wait for a lane change; but also it resets the distance from time to time to check if there is a gap for lane switching upfront instead of behind.

![alt text][state]

#### Result

The link to video is here https://youtu.be/THawmB-wef0. From the video, my average car speed is 49.08/1.05 = 46.74 mph, about 94% efficient compared to the maximum speed 50mph.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

