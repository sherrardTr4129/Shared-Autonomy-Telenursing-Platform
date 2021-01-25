# HRI Web GUI
This software package provides both fontend and backend components of a web-based GUI. The software is currently configured to be run on the same computer as the rest of the ROS enviornment, but this could be easily changed to allow for the GUI software to be run on a seperate machine.

# Dependencies
The backend portion of the application is written in Flask, so it will need to be installed on your system if you don't already have it. This can be done by running the following in a terminal:

```bash
pip install flask
```

The video streaming window within the GUI window needs the [web\_video\_server](http://wiki.ros.org/web_video_server) ROS package to be installed on your system. 

# Usage
First, make sure the ROS master node is running. Then launch the web\_video\_server using the default settings. From here, execute the following command in a new terminal to start the GUI backend:

```bash
rosrun hri_web_gui flask_to_joy.py
```
Once this node successfully starts, open joy.html in a chrome browser and you can start interacting with the GUI. 

Please note that the joy.html file currently assumes that the image topic to display is /cam/camera1/image\_raw1. If that is not the case, please change the web\_video\_server endpoint URL within the joy.html file with the correct topic name. 

If you want to launch both the flask\_to\_joy node and the web\_video\_server node at the same time, you can execute the included launch file: 

```bash
roslaunch hri_web_gui GUI.launch
```

# Published Topics
Please see the table below for an explaination of the topics published by the backend. 


|            Topic           |       Type      |                                                                                                                                                                                                                                                                              Description                                                                                                                                                                                                                                                                              |
|:--------------------------:|:---------------:|:---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| /virtualJoystick           | sensor_msgs/Joy | a sensor\_msgs/Joy message representing the current state of user input. Please see the following for a description of the message components:<br>Axes[0]: Current X-Axis user input (between -1 and 1)<br>Axes[1]: Current Y-Axis user input (between -1 and 1)<br>Axes[2]: Current Z-Axis user input (between -1 and 1)<br>Axes[3]: Current Yaw user input (between -1 and 1)<br>Axes[4]: Current Pitch user input (between -1 and 1)<br>Axes[5]: Current Roll user input (between -1 and 1)<br><br>Buttons[0]: Current gripper open/close state (between 0 and 10) |
| /isInfoEntropyMaximization | std\_msgs/Bool  | Boolean indicating the activation state of the Information Entropy Maximization camera control objective                                                                                                                                                                                                                                                                                                                                                                                                                                                              |
| /isOcclusionAvoidance      | std\_msgs/Bool  | Boolean indicating the activation state of the Occlusion Avoidance camera control objective                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| /isSaliencyMaximization    | std\_msgs/Bool  | Boolean indicating the activation state of the Saliency Maximization camera control objective                                                                                                                                                                                                                                                                                                                                                                                                                                                                         |
