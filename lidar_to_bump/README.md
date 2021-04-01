# LiDAR To Bump Node
This package uses data from the TRINA2 lidar sensor to attempt to identify bump hazards that move result in difficult to escape situations for the remote operator. This node subscribes to the LiDAR point cloud and publishes an angle of the estimated bump hazard with respect to the robot if one exists. Please see the diagram below for a description of the detected bump angle with respect to the robot.

[Detected Bump Angle](https://github.com/sherrardTr4129/Shared-Autonomy-Telenursing-Platform/blob/main/docs/pictures/robotAngle.png)

## Subscribed Topics
This node will attempt to subscribe to a LaserScan topic with the name "/trina2\_1/base\_scan". Please make sure this topic is active before starting this node.

## Published Topics
This node will publish the angle of the detected bump hazards with respect to the robot base frame ( as seen in the above diagram) in degrees as a Float64 message with the topic name "/trina2\_1/bump\_angle". Note that a bump angle of 0 means there is a bump hazard directly in front of the robot, a bump angle of -60 means there is a hazard due left, and a bump angle of 60 means there is a hazard due right. Note that the node may publish angles between these limits as hazards are detected. 
