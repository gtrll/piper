Problem File
======================
Problem settings are provided to any algorithm via a ```.yaml``` file in the [```/problem```](../problem) directory. This file is read by the [```Problem```](../piper/base/problem.cpp) class and creates a [BatchTrajOptimizer](https://github.com/gtrll/gpmp2/blob/master/gpmp2/planner/BatchTrajOptimizer.h) or [ISAM2TrajOptimizer](https://github.com/gtrll/gpmp2/blob/master/gpmp2/planner/ISAM2TrajOptimizer.h) and sets up the optimization settings for the algorithm being used.

For any problem, following is a list of parameters that can be provided. For some of the parameters please see the [GPMP2 documentation](https://github.com/gtrll/gpmp2/blob/master/doc/Parameters.md) to gain more insight:

- ```start_conf```: (optional) start configuration of the arm, read from ```arm_state_topic``` if not passed
- ```start_pose```: (optional) start pose of the base, read from ```base_state_topic``` if not passed
- ```goal_conf```: goal configuration of the arm
- ```goal_pose```: goal pose of the base
- ```sdf_file```: SDF file in the [```/sdf```](../sdf) directory, used for collision avoidance, see [how to create SDF files](sdf.md)
- ```total_time```: total time length of the trajectory in seconds
- ```total_step```: total number of time steps (support states) on the trajectory (includes start and goal)
- ```obs_check_inter```: number of states to be interpolated between any two support states, for collision cost calculation during optimization
- ```control_inter```: number of states to be interpolated between any two support states, to generate executable trajectory for a robot
- ```cost_sigma```: covariance for obstacle factors
- ```epsilon```: safety distance from obstacles in meters
- ```fix_pose_sigma```: small covariance to fix the position part of the state (mainly used for start and goal)
- ```fix_vel_sigma```: small covariance to fix the velocity part of the state (mainly used for start and goal)
- ```Qc```: covariance for GP factors
- ```opt_type```: optimization method to be used, ```LM``` works fine in most cases

---
[Back to Usage](usage.md)

[Back to Documentation home](index.md)
