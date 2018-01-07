# manipulation_planning_suite
This repository contains a set of manipulation (currently only rearrangement) planning algorithms developed at RPL.
It is currently in its early stages of development. 

Available rearrangement planning algorithms:
- NaiveRRT (Standard kinodynamic RRT)
- HybridActionRRT (Kinodynamic RRT that utilizes action primitives to approximately solve the 2BVP)
- OracleRRT (Kinodynamic RRT that utilizes action primitives in the same way as SliceBasedOracleRRT to approximately solve the 2BVP)
- SliceOracleRRT (RRT-style planning algorithm that utilizes a slice view of the problem in combination with learned action primitives)

Parameters for each rearrangement planning algorithm:
**All**:
- timeout - maximum number of seconds to plan (reasonable value is 60s)
- weights - weights of individual entities (objects, robot) in distance function on search space (by default 1.0 for all)
- goal_bias - number in range [0,1] indicating how many times to attempt extending search tree to a goal (0.2 seems a reasonable value)
- target_bias - number in range [0, 1 - robot_bias] that biases the algorithm to attempt moving target objects (0.2 is reasonable again)
- robot_bias - number in range [0, 1 - target_bias] that biases the algorithm to attempt moving the robot (not as important as target_bias)
- object_weights_distance_fn - an optional map that may contain weights used in the distance function for each object. If the object is not in the map,
        the default value (1.0) is used.
**NaiveRRT**:
- num_control_samples - number of control samples
**HybridActionRRT**:
- num_control_samples - number of control samples, must be positive (within 10 is reasonable)
- action_noise - value in [0,1] that indicates randomness of action selection (0.5 is good)
**OracleRRT**:
- state_noise - probability to sample feasible state uniformly rather than from oracle (should be quite small)
**SliceOracleRRT**:
- do_slice_ball_projection - indiciates whether to project a sampled slice to reachable slice in the extension step
- max_pushing_distance - maximum distance an object can be pushed (set in Oracle)
- state_noise - probability to sample feasible state uniformly rather than from oracle (should be quite small, e.g. < 0.01)
- action_noise - probability to sample action uniformly rather than from oracle (should also be small, e.g. < 0.01)

