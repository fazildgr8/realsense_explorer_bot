#!/usr/bin/python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

trigR_flag = False
trigL_flag = False

joy_control = False

prev_state = 0

def callback_joy(msg):
    global trigL_flag,trigR_flag, joy_control, prev_state
    min_max_linear = [-0.25,0.25]
    min_max_angular = [-1.2,1.2]

    axes = msg.axes
    buttons = msg.buttons
    vel = Twist()
    vel.linear.x = translate(axes[1],-1,1,min_max_linear[0],min_max_linear[1])
    vel.angular.z = translate(axes[0],-1,1,min_max_angular[0],min_max_angular[1])

    joint_states = JointState()
    joint_states.name = ['Joint_1','Joint_2','Joint_3']

    joint_3 = 0
    if axes[4]==1:
        trigR_flag=True
    if axes[5]==1:
        trigL_flag=True

    if trigR_flag==True and trigR_flag==True:
        if axes[4]!=1:
            joint_3 = translate(axes[4],-1,1,-1.54,0)
        elif axes[5]!=1:
            joint_3 = translate(axes[5],-1,1,1.54,0)

    pos = [translate(axes[2],-1,1,-1.54,1.54),
           translate(axes[3],-1,1,-1.54,1.54),
           joint_3]
    joint_states.position = pos

    if buttons[11]==1 and prev_state==0:
        if joy_control==True:
            joy_control=False
            rospy.loginfo('Joystick Control [STOPPED]')
        else:
            joy_control=True
            rospy.loginfo('Joystick Control [STARTED]')
    
    prev_state = buttons[11]

    if joy_control==True:

        vel_pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=10)
        vel_pub.publish(vel)

        joint_pub = rospy.Publisher('joint_states_ct', JointState, queue_size=10)
        joint_pub.publish(joint_states)

if __name__ == "__main__":
    rospy.init_node('Joystick_control')
    rospy.Subscriber("joy", Joy, callback_joy)
    rospy.spin()


