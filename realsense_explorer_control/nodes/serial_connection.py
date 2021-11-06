#!/usr/bin/python
from os import lseek
import roslib
import rospy

import serial
import time
import numpy as np
 
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32


port = rospy.get_param("~port","/dev/ttyACM0")
baudrate = rospy.get_param("~baudrate",115200)
imu_frame = rospy.get_param("~imu_frame",'base_link')

connection = serial.Serial(port=port, baudrate=baudrate)
connection.reset_input_buffer()

global l_speed, r_speed
r_speed = 0
l_speed = 0

def encoder_ticks_publish(left_ticks,right_ticks):
    left_tick_pub = rospy.Publisher('lwheel_ticks', Int32, queue_size=10)
    right_tick_pub = rospy.Publisher('rwheel_ticks', Int32, queue_size=10)

    l_ticks = Int32()
    l_ticks.data = left_ticks

    r_ticks = Int32()
    r_ticks.data = right_ticks

    left_tick_pub.publish(l_ticks)
    right_tick_pub.publish(r_ticks)

def data_parser(data_list):
    imu_array = np.zeros(10)

    left_ticks = int(data_list[0])
    right_ticks = int(data_list[1])

    for i in range(len(data_list[2:12])):
        imu_array[i] = float(data_list[i+2])

    return left_ticks, right_ticks, imu_array

def receive_wheel_speeds(l_speed,r_speed):
    send_string = "/"+str(l_speed)+"/"+str(r_speed)+"\n"
    connection.write(send_string.encode('utf-8'))

def imu_message_publish(imu_array):
    imu_msg = Imu()
    
    imu_msg.linear_acceleration.x = imu_array[0]
    imu_msg.linear_acceleration.y = imu_array[1]
    imu_msg.linear_acceleration.z = imu_array[2]
    imu_msg.linear_acceleration_covariance = [-1,0,0,
                                              0,0,0,
                                              0,0,0]

    imu_msg.angular_velocity.x = imu_array[3]
    imu_msg.angular_velocity.y = imu_array[4]
    imu_msg.angular_velocity.z = imu_array[5]
    imu_msg.angular_velocity_covariance = [-1,0,0,
                                           0,0,0,
                                           0,0,0]

    imu_msg.orientation.x = imu_array[6]
    imu_msg.orientation.y = imu_array[7]
    imu_msg.orientation.z = imu_array[8]
    imu_msg.orientation.w = imu_array[9]
    imu_msg.orientation_covariance = [-1,0,0,
                                      0,0,0,
                                      0,0,0]

    imu_msg.header.frame_id = imu_frame
    # imu_msg.header.stamp = rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))

    imu_pub = rospy.Publisher('robot_imu', Imu, queue_size=10)
    imu_pub.publish(imu_msg)


def callback_left_desired_rate(msg):
    global l_speed
    l_speed = msg.data

def callback_right_desired_rate(msg):
    global r_speed
    r_speed = msg.data

if __name__ == '__main__':
    rospy.init_node('serial_connection_RobotHW')
    rospy.Subscriber("lwheel_desired_rate", Int32, callback_left_desired_rate)
    rospy.Subscriber("rwheel_desired_rate", Int32, callback_right_desired_rate)

    while not rospy.is_shutdown():
        data = connection.readline().decode("utf-8")
        # data_list : Left_ticks, Right_ticks, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, QuX, QuY, QuZ, QuW, ignore
        data_list = str(data).split("/")
        if(len(data_list)==13):
            left_ticks, right_ticks, imu_array = data_parser(data_list)
            encoder_ticks_publish(left_ticks,right_ticks)
            imu_message_publish(imu_array)
            receive_wheel_speeds(l_speed,r_speed)

    rospy.spin()