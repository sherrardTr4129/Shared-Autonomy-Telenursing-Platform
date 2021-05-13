# Shared-Autonomy-Telenursing-Platform
This repository contains different packages needed to control the TRINA2 platform in the constructed gazebo simulation enviornment. Please see each individual package's READMD.md file for more in-depth documentation and usage instructions.

# Usage 
Please see each subsection for instructions on how to launch each individual project component.

## Web GUI Application
Before trying to run the web GUI, make sure you have gone through the installation steps outlined [here](https://github.com/sherrardTr4129/Shared-Autonomy-Telenursing-Platform/tree/main/telenursing_web_gui). Once you have done this, you can start the backend nodes and the web_video_server node by running the following command:

```bash
roslaunch telenursing_web_gui GUI.launch
```

Once everything is set up, you can open web/html/MainGUIPage.html and you should be able to control the TRINA2 robot using the joysticks. You should also be able to see a live video stream from both the primary and secondary cameras. 

## Viewpoint Selection and Attainment
The individual viewpoint selection algorithms (information entropy and saliency maximization) can be run individually with the respective commands: 

```bash
rosrun saliency_to_vec saliency_to_vec.py
rosrun info_entropy information_entropy.py 
```

Once both of these nodes come up, you can run the weighted combination vector that will attempt to generate an overall goal viewpoint to be obtained by the secondary camera. This node can be run using the following command:

```bash
rosrun weighted_vector_combination weighted_vector_combination.py
```

You should now be able to see a weighted combination viewpoint represented as a PoseStamped message published over the "/info_and_saliency_avg_pose" topic. From here, you launch the vector_to_movement package that will attempt to take the goal viewpoint as a PoseStamped message and generate the required motion to obtain said viewpoint. 

## Stop-To-Fixate Behavior
This project also implemented a "stop-to-fixate" behavior, in which potential collisions are identified within the LiDAR scan topic, and focused in on using the head camera. This allows for teleoperators to identify potential collisions and escape from entrapment situations. The nodes required to generate this behavior can be run using the following commands:

```bash
rosrun stop_to_fixate headcam_stf.py
rosrun lidar_to_bump lidar_to_bump.py
```

## Running Everything at Once
To run all the aforementioned functionallity at the same time, run the following command: 

```bash
roslaunch weighted_vector_combination viewpoint_select.launch 
roslaunch info_entropy test.launch
```

The bagfiles of the movement are too large to include. If anyone would like access to them feel free to contact any of the others. 
