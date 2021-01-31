#Telenursing TRINA2 Control Package
This package is designed to provide both eye-in-hand camera arm and mobile base control for the TRINA2 platform. This package relies on data from the web GUI package to actuate the robot itself.

## Dependencies
This package assumes you have installed the TRINA-WPI-2.0 stack found [here](https://github.com/hiro-wpi/TRINA-WPI-2.0). The installation steps seen within that repository should install all required dependencies needed here as well.

## Usage
Use the included launch file to bring up all required nodes for control of the TRINA2 platform.

```bash
roslaunch telenursing_trina2_ctrl trina2.launch
```

## Published Topics
No nodes in this package publish topics for use by external subscribers. Nodes within this package do subscribe to topics published from the web GUI package, however.

## Provided Services
Nodes within this package do provide one ROS service which can be called externally to home the left eye-in-hand camera arm. Please see a more detailed description in the table below.

| Service Name    | Parameter Type | Return Type   | Description                                                                                                                                                                                                                                          |
|-----------------|----------------|---------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| /homeLeftArmReq | std\_msgs/Bool  | std\_msgs/Bool | This service will home the left eye-in-hand camera arm when a value of 'True' is sent to the service. The state of the arm's joint values will be updated at this time as well. A value of 'True' will be returned once the homing process completes |

