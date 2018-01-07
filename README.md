# manipulation_planning_suite
This repository contains a set of manipulation (currently only rearrangement) planning algorithms developed at RPL.
It is currently in its early stages of development. 

Available rearrangement planning algorithms:
- NaiveRRT (Standard kinodynamic RRT)
- HybridActionRRT (Kinodynamic RRT that utilizes action primitives to approximately solve the 2BVP)
- OracleRearrangementRRT (Kinodynamic RRT that utilizes learned action primitives to approximately solve the 2BVP)
- SliceBasedOracleRRT (RRT-style planning algorithm that utilizes a slice view of the problem in combination with learned action primitives)

Parameters for each rearrangement planning algorithm:
**All**:
- timeout - maximum number of seconds to plan
- weights - weights of individual entities (objects, robot) in distance function on search space
- goal_bias - number in range [0,1] indicating how many times to attempt extending search tree to a goal
- target_bias - number in range [0, 1 - robot_bias] that biases the algorithm to attempt moving target objects
- robot_bias - number in range [0, 1 - target_bias] that biases the algorithm to attempt moving the robot
**NaiveRRT**:
- num_control_samples - number of control samples
**HybridActionRRT**:
- num_control_samples - number of control samples, must be positive
- p_rand - value in [0,1] that indicates randomness of action selection
**OracleRearrangementRRT**:
- No additional parameters
**SliceBasedOracleRRT**:
- do_slice_ball_projection - indiciates whether to project a sampled slice to reachable slice in the extension step
- max_pushing_distance - maximum distance an object can be pushed (set in Oracle)

