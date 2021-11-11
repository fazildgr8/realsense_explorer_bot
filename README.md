# realsense_explorer_bot
- Autonomous environment exploration  mobile robot which has 3-DOF manipulator with Intel Realsense D435i mounted on a Tracked differential drive mobile  robot fully controlled with ROS in **Jetson Nano** board. 
- The robot is capable of mapping spaces, exploration through RRT, SLAM and 3D pose estimation/localization of objects around it. 
- Robot produces odometry through EKF Filter (ekf_robot_localization) which fuses the IMU (MPU9250) data and the wheel encoder odometry data.
- The custom built Robot uses ROS Realsense D435i RGB-D sensor with dexterity for perception and with Rviz visulaization of Robot State, Point Cloud and the generated map. 
- The Robot uses Realtime Appearance Based Mapping (RTAB-map ROS package) for SLAM appliacation.
- The Robot uses Jetson Nano as its main computer interfaced with the robot hardware (Arduino,Motor-Conttrollers, IMU - MPU9250) through custom Serial interface (not ROS Serrial). 
<img src="https://user-images.githubusercontent.com/24454678/141373757-47f3b3ee-9df0-4290-a63b-7a7bfea64f2a.gif" width="800">
</br>
 ## Robot Representation
 ```
 roslaunch realsense_explorer_description robot_bringup.launch
 ```
<img src="https://user-images.githubusercontent.com/24454678/141372405-30a4fc64-6099-403b-8dc7-63c0af7c291f.jpg" width="800">

- Make sure to change the Global Fixed frame from **base_footprint** to **odomo** after launching the robot_control_ekf node mentioned below.

 ## Diff Drive Robot Control Node Graph with EKF Localization
 ```
 roslaunch realsense_explorer_control robot_control_ekf.launch
 ```
 ![control_node_graph_main](https://user-images.githubusercontent.com/24454678/141372490-8e740dd8-0715-42e8-beb0-8e0d1f50c2b9.png)
 - **/serial_connection_RobotHW** - The Hardware Interface node through serial port which communicates with the microcontroller to get the Robot IMU sensor data, two wheel encoder readings and publishes the appropriate messages to other nodes. It also sends the desired wheel rates for differential drive back to the microcontroller. (Arduino Uno Code : [realsense_explorer_bot/realsense_explorer_control/arduino/robot_diff_drive.ino](https://github.com/fazildgr8/realsense_explorer_bot/blob/main/realsense_explorer_control/arduino/robot_diff_drive.ino))
 - **/diff_drive_controller** - The differential drive inverse kinematics node which receives the /cmd_vel (Linear x vel, Angular z vel) and produces the required wheel speeds in encoder ticks per second.
 - **/odom_publisher** - The node produces odometry through reading the wheel encoders
 - **/robot_ekf_localization** - The node fuses the odometry through wheel encoders and the Robot IMU sensor data to produce EKF filtered odometry of the robot.
 - **/jointState_to_servos** - The node which converts three servo joint position from the /joint_state_publisher to 3-DOF manipulator hardware controlled by servo motor driver connected directly to Jetson Nano through I2C communication.

## The Robot's Perception Setup can be forked from [github.com/fazildgr8/realsense_bot](https://github.com/fazildgr8/realsense_bot)
<img src="https://user-images.githubusercontent.com/24454678/138940187-ebf82bbe-8ebc-4dda-8a9d-005dba85545d.png" width="800">

### [Updates Coming Soon on Navigation and Mapping] ###
