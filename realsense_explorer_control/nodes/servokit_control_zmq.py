#!/usr/bin/python

import roslib
import rospy
import numpy as np
import zmq
import math
from sensor_msgs.msg import JointState
global joint_poses, socket

joint_poses = [90,90,90]

port = "5556"

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:%s" % port)

def callback(msg):
	global joint_poses, socket
	joint_poses[0] = math.degrees(msg.position[0])+90
	joint_poses[1] = math.degrees(-msg.position[1])+90
	joint_poses[2] = math.degrees(msg.position[2])+90
	x_arrstr = np.char.mod('%f', np.array(joint_poses))
	messagedata = ",".join(x_arrstr)
	socket.send(messagedata)

if __name__ == "__main__":
	rospy.init_node('JointState_Servo')
	joint_poses[0] = 90
	joint_poses[1] = 90
	joint_poses[2] = 90
	rospy.Subscriber("joint_states", JointState, callback)
	rospy.spin()
