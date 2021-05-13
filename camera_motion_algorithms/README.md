# Camera Motion Algorithms
This folder contains all the camera motion algorithms produced for this project (i.e. wrist roll reduction, motion buffer, etc) as ROS packages.

## Stop-to-Fixate Package
The stop-to-fixate behavior is launched through the stop_to_fixate package. The node produces the vector indicating the yaw and pitch that the head camera should move to to have an optimal viewing angle of the imminent collision risk location.

To run the stop-to-fixate behavior, start by running the lidar-to-bump node, which produces the LiDAR "bump" sensor reading:
```bash
rosrun lidar_to_bump lidar_to_bump.py
```
Then, to produce the actual stop-to-fixate vector, run the stop-to-fixate node, which subscribes to the LiDAR reading:
```bash
rosrun stop_to_fixate headcam_stf.py
```
The actual motion is carried out by the vector_to_movement package, so launch that package next ->

## Vector to Movement Package
This package contains the functionality for movment based on the weighted viewpoint selection vector or the stop-to-fixate interrupt vector. The package decides to move either the head camera or the left arm-in-hand camera based on which is the secondary camera at the moment.

To launch the actual viewpoint attainment component of our project, launch:
```bash
roslaunch vector_to_movement vector_to_movement.launch
```

As long as the viewpoint selection nodes are running, the robot should begin moving based on the produced vectors, or if the robot base approaches too close to an obstacle, the head camera should begin moving to observe the potential collision. 