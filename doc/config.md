Robot Configuration File
======================
Robot configuration and settings are provided to any algorithm via a ```.yaml``` file in the [```/config```](../config) directory. Part of this file is read by the [```Robot```](../piper/base/robot.cpp) class which creates a GPMP2 [```RobotModel```](https://github.com/gtrll/gpmp2/blob/master/gpmp2/kinematics/RobotModel.h), and the remaining part is read by the [```Traj```](../piper/base/traj.cpp) class which assigns settings for controlling the robot through ROS.

For any robot, following is a list of parameters that can be provided:

- ```mobile_base```: boolean flag to be set to _True_ if the robot has a mobile base and is being used by any algorithm
- ```DOF```: total degree-of-freedom (DOF) of the robot. For example, a mobile manipulator with a 6 DOF arm has a total of 9 DOF (3 for base + 6 for arm)
- ```arm_base```: pose of the arm's base relative to the base of the robot if using the full robot with the base, otherwise the absolute pose of the the arm's base relative to the world coordinates
- ```DH```: DH parameters of the robot (arm), where ```theta``` is the additive bias applied to any joint and ```theta_neg``` is specified if any joint angles needs to flip their sign
- ```spheres```: for collision checking the robot body is approximated with a collection of spheres, where any sphere of radius ```r``` is located at ```(x, y, z)``` relative to any joint ```j```, and this information is provided with individual lists ```js```, ```xs```, ```ys```, ```zs```, and ```rs``` in order of the spheres
- ```arm_joint_names```: an array of strings with the names of all the joints on the arm in order
- ```sensor_arm_sigma```: variance of sensor model for arm state measurement
- ```sensor_base_sigma```: variance of sensor model for base state measurement
- ```trajectory_control_topic```: topic name for a action client on the robot API that accepts FollowJointTrajectoryAction ROS message type and is used to execute any trajectory on the robot
- ```est_traj_pub_topic```: (optional) publishes estimated trajectory to this topic
- ```plan_traj_pub_topic```: (optional) publishes planned trajectory to this topic
- ```arm_state_topic```: current arm state can be read from this topic and is published by the robot API
- ```base_state_topic```: current base state can be read from this topic and is published by the robot API

To create a robot config file for your own robot you simply need its DH parameters, an API to execute passed trajectories and publish state information and a list of spheres that well approximate the robot's body. The spheres, for example, in [GPMP2](https://github.com/gtrll/gpmp2) were put in the robot ```.xml``` files and were set by loading the robot model in OpenRAVE or Matlab and then manually working out the sphere locations. Some examples of robot config files are already included in the [```/config```](../config) directory.

---
[Back to Usage](usage.md)

[Back to Documentation home](index.md)
