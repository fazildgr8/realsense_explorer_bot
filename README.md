# realsense_explorer_bot
- Autonomous environment exploration  mobile robot which has 3-DOF manipulator with Intel Realsense D435i mounted on a Tracked differential drive mobile  robot fully controlled with ROS in **Jetson Nano** board. 
- The robot is capable of mapping spaces, exploration through RRT, SLAM and 3D pose estimation/localization of objects around it. 
- Robot produces odometry through EKF Filter (ekf_robot_localization) which fuses the IMU (MPU6050) data and the wheel encoder odometry data.
- The custom built Robot uses Realsense D435i RGB-D sensor with dexterity for perception and with Rviz visulaization of Robot State, Point Cloud and the generated map. 
- The Robot uses Realtime Appearance Based Mapping (RTAB-map ROS package) for SLAM appliacation.
- The Robot uses Jetson Nano as its main computer interfaced with the robot hardware (Arduino,Motor-Conttrollers, MPU6050) through custom Serial interface (not ROS Serrial). 

 ## Robot Description
 The [realsense_explorer_description](https://github.com/fazildgr8/realsense_explorer_bot/tree/main/realsense_explorer_description) packae consists of the Robot's URDF files, launch file for loading robot description, Rviz config,robot state and joint state publishers for the robot.
 ```
 roslaunch realsense_explorer_description robot_bringup.launch
 ```
<img src="https://user-images.githubusercontent.com/24454678/141372405-30a4fc64-6099-403b-8dc7-63c0af7c291f.jpg" width="600">

- Make sure to change the Global Fixed frame from **base_footprint** to **odomo or map** after launching the robot_control_ekf node mentioned below.

 ## Robot Control Node Graph with EKF Localization
 The [realsense_explorer_control](https://github.com/fazildgr8/realsense_explorer_bot/tree/main/realsense_explorer_control) package consists of the nodes and launch file required to interface the robot hardware with ROS.
 ```
 roslaunch realsense_explorer_control robot_control_ekf.launch
 ```
 ![control_node_graph_main](https://user-images.githubusercontent.com/24454678/141372490-8e740dd8-0715-42e8-beb0-8e0d1f50c2b9.png)
 - **/serial_connection_RobotHW** - The Hardware Interface node through serial port which communicates with the microcontroller to get the Robot IMU sensor data, two wheel encoder readings and publishes the appropriate messages to other nodes. It also sends the desired wheel rates for differential drive back to the microcontroller. (Arduino Uno Code : [realsense_explorer_bot/realsense_explorer_control/arduino/robot_diff_drive.ino](https://github.com/fazildgr8/realsense_explorer_bot/blob/main/realsense_explorer_control/arduino/robot_diff_drive.ino))
 - **/diff_drive_controller** - The differential drive inverse kinematics node which receives the /cmd_vel (Linear x vel, Angular z vel) and produces the required wheel speeds in encoder ticks per second.
 - **/odom_publisher** - The node produces odometry through reading the wheel encoders
 - **/robot_ekf_localization** - The node fuses the odometry through wheel encoders and the Robot IMU sensor data to produce EKF filtered odometry of the robot.
 - **/jointState_to_servos** - The node which converts three servo joint position from the /joint_state_publisher to PWM signals for servo motors of the 3-DOF manipulator controlled by servo motor driver connected directly to Jetson Nano through I2C communication.

## Robot 3D Perception with Multi Object Tracking
The [realsense_explorer_perception](https://github.com/fazildgr8/realsense_explorer_bot/tree/main/realsense_explorer_perception) package consists of all the peception related nodes/launch for 3D multi object tracking, PCL cloud stream and RTAB Mapping. 
The following sequence of launch is to be executed for Multi Object 3D tracking.
```
roslaunch realsense_explorer_perception start_rs_camera.launch filters:=pointcloud
```

https://user-images.githubusercontent.com/24454678/141846545-898a943d-7062-4edb-8ef4-2a3a3966f503.mp4

##### The Robot's Objet Tracking method can be understod from [github.com/fazildgr8/realsense_bot](https://github.com/fazildgr8/realsense_bot)
<img src="https://user-images.githubusercontent.com/24454678/138940187-ebf82bbe-8ebc-4dda-8a9d-005dba85545d.png" width="400">

## Robot RTAB Mapping 3D Space/ 2D Grid Map Demo


https://user-images.githubusercontent.com/24454678/141847152-034f0f2c-e753-4d1a-9389-505e2fbc505f.mp4


### Notes on remote Robot Control 
- The robot description and the Rviz visulaization can be brought up in a master Desktop computer running a ROS core.
- The Robot control and Localization launch should be running in the Jetson Nano with it's ROS Master URI set to the Desktop computer's IP address.
- The robot's movement can be controlled by Robot steering in RQT Gui and the 3-DOF manipulator through the Joint state publisher GUI.  
### [Updates Coming Soon on using Navigation Stack with RRT space exploration]
- The **RRT exploration Simulation** can be found in [github.com/fazildgr8/ros_autonomous_slam](https://github.com/fazildgr8/ros_autonomous_slam)
![RRT](https://user-images.githubusercontent.com/24454678/141375945-3afb0fb7-ff4a-4bb7-b20c-0fbf3e326033.gif)

