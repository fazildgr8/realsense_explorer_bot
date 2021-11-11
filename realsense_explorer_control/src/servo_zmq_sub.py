#!/usr/bin/python3

from adafruit_servokit import ServoKit
import zmq

port = "5556"

kit = ServoKit(channels=16,frequency=50)

# Socket to talk to server
context = zmq.Context()
socket = context.socket(zmq.SUB)

socket.connect ("tcp://localhost:%s" % port)

socket.setsockopt_string(zmq.SUBSCRIBE,"")

def string_parser(string):
    data = string.split(",")
    joint_1 = float(data[0][2:])
    joint_2 = float(data[1])
    joint_3 = float(data[2][:-1])

    return [joint_1,joint_2,joint_3]

if __name__ == "__main__":
    while True:
        string = str(socket.recv())
        # print(string_parser(string))
        joints = string_parser(string)
        kit.servo[0].angle = int(joints[0])
        kit.servo[1].angle = int(joints[1])
        kit.servo[2].angle = int(joints[2])




