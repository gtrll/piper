Signed Distance Field File
======================
A Signed Distance Field (SDF) file is used for collision avoidance and can be loaded in to any algorithm from the [```/sdf```](../sdf) directory, by passing its filename in the problem file. The [```Problem```](../piper/base/problem.cpp) class create a GPMP2 [```SignedDistanceField```](https://github.com/gtrll/gpmp2/blob/master/gpmp2/obstacle/SignedDistanceField.h) object by reading it with the [```loadSDF()```](https://github.com/gtrll/gpmp2/blob/master/gpmp2/obstacle/SignedDistanceField.h) or [```readSDFvolfile()```](https://github.com/gtrll/gpmp2/blob/master/gpmp2/utils/fileUtils.cpp) function.

For any of your environments you can create your own SDF file with any of the following options:

- **Matlab**: A Matlab wrapper is already included with GPMP2. Create your environment (occupancy grid) in Matlab and then generate a SDF by following this [example](https://github.com/gtrll/gpmp2/blob/master/matlab/SaveSDFExample.m).
- **OpenRAVE**: The GPMP2 OpenRAVE plugin - [orgpmp2](https://github.com/gtrll/orgpmp2) can be used to create a SDF. First create your environment with a ```.xml``` file and then follow this [example](https://github.com/gtrll/orgpmp2/blob/master/examples/save_sdf_pr2.py). If you are using only the arm on a mobile manipulator make sure to keep the body of the robot minus the arm in the environment when generating the SDF.
- **Real sensor data**: You can write you own library to map an environment with real sensor data (LIDAR, camera, etc) and create an occupancy grid. Then convert that to the [```SignedDistanceField```](https://github.com/gtrll/gpmp2/blob/master/gpmp2/obstacle/SignedDistanceField.h) data type and save it using the [```saveSDF()```](https://github.com/gtrll/gpmp2/blob/master/gpmp2/obstacle/SignedDistanceField.h) function. A similar process is described in the [STEAP](http://www.cc.gatech.edu/~bboots3/files/STEAP.pdf) publication (see section IV D).

---
[Back to Problem file](problem.md)

[Back to Usage](usage.md)

[Back to Documentation home](index.md)
