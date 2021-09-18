# Autonomous Flight Planning and Control

This repository contains packages that enable the autonomous flight planning and control of a simulated quadrotor system. It was inspired by the content in the Optimal and Learning Based Control course, [AA 203](http://asl.stanford.edu/aa203/).

## Package List

This repository consists of several ROS packages
- navbotsim - simulates the world that the Navbot lives in
- navbot_plan - contains planning libraries for autonomous flight
- navbot - Nonlinear model simulation and linear MPC of 12D quadrotor navbot

## Getting Started

### Dependencies

* ROS Noetic (desktop-full)
* Python 3
* CVXPY
* Numpy
* Scipy

### Installing

These directions assume you have completed installation and setup of ROS Noetic (desktop-full) and Python3 with pip

* Install cvxpy using the command `pip install cvxpy`
* install numpy using the command `pip install numpy`
* install scipy using the command `pip install scipy`
* Clone this repository into the src directory of your workspace
* Build and source your workspace

### Executing program

#### Simulation

[Autonomous Flight Planning and Control](https://youtu.be/YIE2_jggq4w)

Simulate the autnomous path planning and control by running the folowing command:

`roslaunch navbot fly.launch`

## TODO

There are opportunities several opportunities for future development.

### Navbot Plan

The path planning algorithms implemented take a long time to execute.
* The current line of sight algorithm uses interpolation and is extremely inefficient-- implement time efficient line of sight algorithm
* A*/theta* are not very fast algorithms. Consider alternatives for faster replanning (perhaps an RRT?)

Memory use can be further optimized.
* Transition from using set/dictionary combination to solely a dictionary (sorting still necessary for A* variants)

### Navbot
The simulation is not very realistic. It currently works by assuming the state is known, calculating the control signal (assuming it can be completed within 0.1 seconds), and then once the control has been successfully computed, the state is simulated forward by 0.1 seconds.
* We can be more realistic by adding noise to the state input to the controller-- the true state is not known
* Separate the nonlinear model simulation from the controller completely into separate nodes that communicate by publishing/subscribing to a topic.
* Port the code to C++ for much faster execution-- consider NLopt or Acado for MPC

The controller does not follow the path as closely as might be desired, particularly after switching between waypoints.
* Velocity tracking could smooth transitions between path waypoints
* Adjust objective function to punish deviation from straight line between waypoints (perhaps could be done by interpolating goals between path waypoints).

## Authors

Nathaniel Nyberg

## Acknowledgments

Inspiration, code snippets, etc.

* I was inspired to do this project and applied concepts from the Optimal and Learning Based Control course, [AA 203](http://asl.stanford.edu/aa203/).
* I followed the mathematical model of a Quadrotor from 1.

1. Ru, P.; Subbarao, K. Nonlinear Model Predictive Control for Unmanned Aerial Vehicles. Aerospace 2017, 4, 31. https://doi.org/10.3390/aerospace4020031




