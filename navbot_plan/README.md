# Navbot Plan
Path planning for the navbot
* `roslaunch navbot_plan plan.launch` to start and visualize the planner.

## Needs Work/Opportunities for Improvement

The path planning algorithms implemented take a long time to execute.
* The current line of sight algorithm uses interpolation and is extremely inefficient-- implement time efficient line of sight algorithm
* A*/theta* are not very fast algorithms. Consider alternatives for faster replanning (perhaps an RRT?)

Memory use can be further optimized.
* Transition from using set/dictionary combination to solely a dictionary (sorting still necessary for A* variants)