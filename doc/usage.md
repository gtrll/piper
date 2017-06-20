Usage
==============
Any algorithm can be run on a simulated or real robot with a ```.launch``` file (see some examples in the [```/launch```](../launch) directory) via ROS using the following command
  
  ```bash
  roslaunch piper myAlgo_interface.launch robot:=myRobot problem:=myProblem
  ```

- ```myAlgo``` is the algorithm you want to run on the robot. Interfaces to all the currently supported algorithms can be found in the [```/piper```](../piper) directory. To add your own algorithm, see the [development guide](development.md).
- ```myRobot``` is a ```.yaml``` file that specifies the robot parameters and settings. They are located in the [```/config```](../config) directory. See [how to use or write config files](config.md) to understand settings of robots currently included or easily write one for your own robot.
- ```myProblem``` is a ```.yaml``` file that specifies the problem to be solved and its settings. They are located in the [```/problem```](../problem) directory. See [how to write problem files](problem.md) to make use of any algorithms that are included.

---
[Back to Documentation home](index.md)
