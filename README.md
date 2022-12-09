# Path Planning in C++

The seventh project in the Udacity Self-Driving Car Engineer Nanodegree: implementing path planning in C++ successfully implement path planning and navigate a highway driving scenario.

## Description

The this project consists of the following components/steps: TODO

1. First component TODO
2. Second component TODO

I'll go through them now in slightly more detail.

### 1. First component TODO

A thorough description. TODO

### 2. Second component TODO

Another thorough description. TODO

## Requirements

- cmake >= 3.10
- make >= 4.1
- gcc/g++ >= 5.4
- uWebSockets
- [Eigen](https://eigen.tuxfamily.org/index.php)

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
2. Run the simulator and select TODO

When you do this, the particle filter executable should report:
```
Connected!!!
```
3. Click start. You should see TODO

![Kidnapped Robot Simulation](TODO)

> NOTE: If you want to run the highway scenario again, click the RESET button but restart the particle filter executable before clicking START again!
