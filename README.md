# realsense_explorer_bot
- Autonomous environment exploration  mobile robot which has 3-DOF manipulator with Intel Realsense D435i mounted on a Tracked differential drive mobile  robot fully controlled with ROS in **Jetson Nano** board. 
- The robot is capable of mapping spaces, exploration through RRT, SLAM and 3D pose estimation/localization of objects around it. 
- Robot produces odometry through EKF Filter (ekf_robot_localization) which fuses the IMU (MPU9250) data and the wheel encoder odometry data.
- The custom built Robot uses ROS Realsense D435i RGB-D sensor with dexterity for perception and with Rviz visulaization of Robot State, Point Cloud and the generated map. 
- The Robot uses Realtime Appearance Based Mapping (RTAB-map ROS package) for SLAM appliacation.
- The Robot uses Jetson Nano as its main computer interfaced with the robot hardware (Arduino,Motor-Conttrollers, IMU - MPU9250) through custom Serial interface (not ROS Serrial). 

 ## Robot Representation
 ```
 roslaunch realsense_explorer_description robot_bringup.launch
 ```
![realsense_explorer](https://user-images.githubusercontent.com/24454678/141041838-f460d1d7-6816-4f1c-8b05-c221afe544ad.png)

 ## Diff Drive Robot Control Node Graph with EKF
 ```
 roslaunch realsense_explorer_control robot_control_ekf.launch
 ```
![robot_graph](https://user-images.githubusercontent.com/24454678/141041852-2e2d380f-32ec-4bdf-97c0-30d47c196c60.png)

## The Robot's Perception Setup can be found in [github.com/fazildgr8/realsense_bot](https://github.com/fazildgr8/realsense_bot)
<img src="https://user-images.githubusercontent.com/24454678/138940187-ebf82bbe-8ebc-4dda-8a9d-005dba85545d.png" width="800">

### [Updates Coming Soon on Navigation and Mapping] ###
