# Navbot
Nonlinear model simulation and linear MPC of 12D quadrotor navbot
* `roslaunch navbot fly.launch` to run the simulation + control
* `roslaunch navbot fly.launch enable_replan:=true` to run the simulation + control with replanning

## Needs Work/Opportunities for Future Development
The simulation is not very realistic. It currently works by assuming the state is known, calculating the control signal (assuming it can be completed within 0.1 seconds), and then once the control has been successfully computed, the state is simulated forward by 0.1 seconds.
* We can be more realistic by adding noise to the state input to the controller-- the true state is not known
* Separate the nonlinear model simulation from the controller completely into separate nodes that communicate by publishing/subscribing to a topic.
* Port the code to C++ for much faster execution-- consider NLopt or Acado for MPC

The controller does not follow the path as closely as might be desired, particularly after switching between waypoints.
* Velocity tracking could smooth transitions between path waypoints
* Adjust objective function to punish deviation from straight line between waypoints (perhaps could be done by interpolating goals between path waypoints).
