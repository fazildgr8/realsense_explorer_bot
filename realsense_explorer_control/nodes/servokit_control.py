#!/usr/bin/python3

from adafruit_servokit import ServoKit
import roslib
import rospy
import math
from sensor_msgs.msg import JointState
global joint_poses

joint_poses = [90,90,90]

kit = ServoKit(channels=16,frequency=50)

def callback(msg):
	global joint_poses, servo
	kit.servo[0].angle = math.degrees(msg.position[0])+90
	kit.servo[1].angle = math.degrees(-msg.position[1])+90
	kit.servo[2].angle = math.degrees(msg.position[2])+90

if __name__ == "__main__":
	rospy.init_node('JointState_Servo')
	kit.servo[0].angle = 90
	kit.servo[1].angle = 90
	kit.servo[2].angle = 90
	rospy.Subscriber("joint_states", JointState, callback)
	rospy.spin()
