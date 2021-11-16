# realsense_explorer_bot
- Autonomous environment exploration  mobile robot which has 3-DOF manipulator with Intel Realsense D435i mounted on a Tracked differential drive mobile  robot fully controlled with ROS in **Jetson Nano** board. 
- The robot is capable of mapping spaces, exploration through RRT, SLAM and 3D pose estimation/localization of objects around it. 
- Robot produces odometry through EKF Filter (ekf_robot_localization) which fuses the IMU (MPU6050) data and the wheel encoder odometry data.
- The Robot uses Realsense D435i RGB-D sensor with dexterity for perception and with Rviz visulaization of Robot State, Point Cloud and the generated map. 
- The Robot uses Realtime Appearance Based Mapping (RTAB-map ROS package) for SLAM appliacation.
- The Robot uses Jetson Nano as its main computer interfaced with the robot hardware (Arduino, Motor Controllers, MPU6050, Servo Controller) through custom Serial interface (not ROS Serrial).
- PID controller implemented within the microcontroller to set the desired speeds to each motors.<br/> 

**Multi Application Video**


https://user-images.githubusercontent.com/24454678/141891883-860446a1-e5da-44ad-9525-07fee55ecc75.mp4




 ## Robot Description
 The [realsense_explorer_description](https://github.com/fazildgr8/realsense_explorer_bot/tree/main/realsense_explorer_description) packae consists of the Robot's URDF files, launch file for loading robot description, Rviz config,robot state and joint state publishers for the robot.
 ```
 roslaunch realsense_explorer_description robot_bringup.launch
 ```
<img src="https://user-images.githubusercontent.com/24454678/141372405-30a4fc64-6099-403b-8dc7-63c0af7c291f.jpg" width="800">

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
- Start RGB-D Stream from Realsense D435i
```
roslaunch realsense_explorer_perception start_rs_camera.launch filters:=pointcloud
```
- Start YoloV3 based Object Detection over the RGB image.
```
roslaunch realsense_explorer_perception object_localization.launch 
```
- Start Multi Object Tracker node which projects detetected objects in 3D space and broadcasts to the TF tree.
```
rosrun realsense_explorer_perception yolo_MultiObject_track.py
```
**Multi Object Tracking Demo Video**

https://user-images.githubusercontent.com/24454678/141846545-898a943d-7062-4edb-8ef4-2a3a3966f503.mp4

- Add Objects to be tracked while initiating the `MultiObject_Tracker` class object in [yolo_MultiObject_track.py](https://github.com/fazildgr8/realsense_explorer_bot/blob/main/realsense_explorer_perception/nodes/yolo_MultiObject_track.py) node script at Line 96.
```
# Example
if __name__=='__main__':
    rospy.init_node('MultiObject_Tracker')
    rate = rospy.Rate(10.0)
    # Add your Objects
    tracker = MultiObject_Tracker(obejcts_to_track = ['person','cup','bottle','chair'])
    tracker.start_subscribers()
    time.sleep(3)
    while not rospy.is_shutdown():
        tracker.objects_tf_send()
        # rate.sleep()
    rospy.spin()
```

##### The Robot's Objet Tracking method can be understod in from [github.com/fazildgr8/realsense_bot](https://github.com/fazildgr8/realsense_bot)
<img src="https://user-images.githubusercontent.com/24454678/138940187-ebf82bbe-8ebc-4dda-8a9d-005dba85545d.png" width="400">

## Robot RTAB Mapping 3D Space/ 2D Grid Map Demo
The Robot uses Realtime Appearance Based Mapping (rtabmap_ros) for mapping 3D spaces/2D occupancy grid maps. Further RTAB Map also works in localization mode.
- Start RTAB Map for Mapping
```
roslaunch realsense_explorer_perception rtab_mapping.launch
```
- Start RTAB Map for Localization
```
roslaunch realsense_explorer_perception rtab_mapping.launch localization:=true
```
**RTAB Mapping Demo Video**

https://user-images.githubusercontent.com/24454678/141847152-034f0f2c-e753-4d1a-9389-505e2fbc505f.mp4

### [Updates Coming Soon on using Navigation Stack with RRT space exploration]
- The **RRT exploration Simulation** can be found in [github.com/fazildgr8/ros_autonomous_slam](https://github.com/fazildgr8/ros_autonomous_slam)
![RRT](https://user-images.githubusercontent.com/24454678/141375945-3afb0fb7-ff4a-4bb7-b20c-0fbf3e326033.gif)

### Robot Package Dependencies
- [Librealsense (Realsense 2 SDK)](https://github.com/IntelRealSense/librealsense) : For Jetson Devices(ARM) build and install from source with Python dependencies to support Realsense D435i camera with Jetson.
- [IntelRealsense ROS Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [Darknet ROS (YoloV3)](https://github.com/leggedrobotics/darknet_ros)
- [imu_tools](http://wiki.ros.org/imu_tools)
- [robot_localization](http://wiki.ros.org/robot_localization)
- [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)
- [OpenCV Python](https://pypi.org/project/opencv-python/)

### Notes on remote Robot Control 
- The robot description and the Rviz visulaization can be brought up in a master Desktop computer running a ROS core(Or Vice Versa).
- The Robot control and Localization launch should be running in the Jetson Nano with it's ROS Master URI set to the Desktop computer's IP address (Or Vice Versa).
- The robot's 3-DOF manipulator can be controlled through the Joint state publisher GUI or nodes publishing Joint angles to /joint_states_ct topic.
