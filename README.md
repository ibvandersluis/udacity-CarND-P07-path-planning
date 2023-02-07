# Path Planning in C++

The seventh project in the Udacity Self-Driving Car Engineer Nanodegree: implementing path planning in C++ successfully implement path planning and navigate a highway driving scenario.

## A Note to the Reviewer

Hello! Sorry for any difficulty building the project.
I like to reduce the amount of third party code I include in my projects if I know it can be installed locally.
I've provided descriptions on how to do this below, but depending on your setup that may still not be ideal.
I noticed the virtual workspace is running on Ubuntu 16.04 (yikes) and did not include package manager instructions for installing dependencies on systems older than Ubuntu 18.04. It can be installed manually, but that likely isn't worth your time just for grading this.

Likely because of the age of the system, the installed version of Eigen in the workspace is behind the one I'm using (3.2.92 vs 3.4.0).

I also add compiler flags that make compilation pedantic about warnings and errors, which I've turned off for now.

I'm resovlging this temporarily for grading by making some changes to make it more friendly to Ubuntu 16:
  - I included a local copy of nlohmann-json since the workspace doesn't have it installed
  - I turned off pedantic compiler flags that were causing build errors from the older version of Eigen

I have cloned my repo into the workspace and have successfully built there to make sure it works.

Hopefully that fixes everything.
If there are any further issues I will submit a screen recording of the vehicle completing a loop of the track.

## Description

The this project consists of the following components:

1. Trajectory generation
2. Obstacle detection
3. Behavior Control

### 1. Trajectory Generation

The vehicle starts in the middle lane of a freeway, so a safe assumption is that we should start by driving forward and staying in our lane.
Forward, however, is not a consistent notion on the map; at least not in terms of Cartesian coordinates.
For the purposes of this project, it therefore makes sense to deal in Frenet coordinates, which use the meandering path of the road as the basis for the axis and gives us coordinates in terms of S and D.
S describes the meter measurement in the longitudinal direction, while D measures the lateral distance from the center of the road.

Trajectory generation is done by determining the car's current position and several widely-spaced points along its desired path in Frenet coordinates.
After converting these Frenet coordinates to Cartesian coordinates, we can generate a spline from which we can extract more dense waypoints and space them appropriately to give us the speed that we want.

### 2. Obstacle Detection

Information about obstacles (in this case, other vehicles) is given to us through the sensor fusion data from the simulator.
How the vehicle responds to another vehicle on the road depends on two main things: lane placement and longitudinal proximity.
The rules for responding to a nearby vehicle are different depending on whether the vehicle is in the same lane as the ego vehile, an adjacent lane, or father away.

If a detected vehicle is in the same lane as the ego vehicle, its position is checked to see if it is closer than a certain threshold ahead (in meters).
If it is, the `car_ahead` flag is set to `true`.
An additionall flag is set to signal that the ego vehicle should reduce its speed.
If the detected vehicle is in an adjacent lane, its position is checked to see if it is closer than a certain threshold *both ahead and behind*.
If it is, the relative lane is flagged as occupied.
If the detected vehicle is more than one lane away from the ego vehicle, it is ignored, as it is unlikely to interfere with the path of the ego vehicle.

This categorization is implemented in the following code:

```cpp
auto lane_diff = check_car_lane - target_lane;

switch (lane_diff) {
  case 0:
    // Vehicle is in our lane
    if (check_car_s > start_s && check_car_s - start_s < 30) {
      car_ahead = true;
      reduce_speed = true;
    }
    break;
  case 1:
    // Vehicle is in the lane on the right
    if (abs(check_car_s - start_s) < 30) {
      car_right = true;
    }
    break;
  case -1:
    // Vehicle is in the lane on our left
    if (abs(check_car_s - start_s) < 30) {
      car_left = true;
    }
    break;
  default:
    // Car is not in an adjacent lane
    break;
}
```

### Behavior Control

Finally, once the vehicle is travelling along a trajectory and has detected nearby vehicles, the final step is to determine what to do with this information.
The resulting decision must be safe and comfortable for the passenger, so the vehicle should not take action that would lead it to collide with another vehicle, or do suddenly change its motion in an unpleasant way.

The behavior control for the vehicle simply checks if a slower vehicle has been detected in front of it.
Then, a decision is made based on lane position:

  - If the vehicle is not in the leftmost lane and the lane to its left is clear, it will change lanes to the left.
  - Similarly, if the vehicle is not in the rightmost lane and the lane to its right is clear, it will change lanes to the right.
  - If the vehicle is in the middle lane, it will prioritize passing on the left to passing on the right.
  - If a slower vehicle is ahead and there is no lane to change to safely, the vehicle will simply reduce its speed and stay in the same lane. This will then be recomputed at the next iteration, and if a lane becomes available, the car will then switch to it.

![Changing Lanes](https://user-images.githubusercontent.com/14826664/217096795-8fe9dc6d-4d41-40ed-bb1f-597240ff4126.png)

The logic for the behavior controller looks like this in the code:

```cpp
if (car_ahead) {
  // Attempt lange change if slower vehicle ahead
  // Prioritize passing on the left and change lanes if safe
  if (target_lane > 0 && !car_left) {
    target_lane -= 1;
  } else if (target_lane < 2 && !car_right) {
    target_lane += 1;
  }
}
```

Using these rules, the ego vehicle will safely complete a circuit of the track.

## Requirements

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- [uWebSockets](https://github.com/uWebSockets/uWebSockets) @ `e94b6e1`
- [nlohmann-json](https://github.com/nlohmann/json)
- [Eigen](https://eigen.tuxfamily.org/index.php) >= 3.4

To install uWebSockets, follow these instructions:
```
git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
```

To install nlohmann-json and Eigen, you have serveral options. You can:

1. Install using a package manager (easiest)

For Ubuntu:

```bash
sudo apt install nlohmann-json3-dev libeigen3-dev # use nlohmann-json-dev for Ubuntu 18.04
```

For Fedora:

```bash
sudo dnf install json-devel eigen3-devel
```

For MacOS:
```bash
brew install nlohmann-json eigen
```

2. Download manually

```bash
wget https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
tar -xvf json.tar.xz
sudo cp -r json/include/nlohmann /usr/include/
rm -rf json json.tar.xz
```

Eigen can be downloaded manually from the website [here](https://eigen.tuxfamily.org/index.php).

## Build instructions

1. Clone this repo
```console
git clone https://github.com/ibvandersluis/udacity-CarND-P07-path-planning.git
cd udacity-CarND-P07-path-planning
```
2. Build the project
```console
mkdir build && cd build
cmake ..
make
```

## Run

To run this project as intended, you will need the [Term 3 Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

1. Run the particle filter
```console
./path_planning
```
You should get back:
```
Listening to port 4567
```
2. Run the simulator and select the path planning project

When you do this, the particle filter executable should report:
```
Connected!!!
```
3. The vehicle should start moving automatically, starting from stopped and slowly increasing its speed until it is close to the speed limit.

![Path Planning Simulation](https://user-images.githubusercontent.com/14826664/217097418-4c27159a-2005-4fda-b83d-48755101d3be.png)
