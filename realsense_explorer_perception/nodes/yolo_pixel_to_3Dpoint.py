#!/usr/bin/python
import roslib
import rospy
import tf
from darknet_ros_msgs.msg import BoundingBoxes,ObjectCount
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
from rospy.numpy_msg import numpy_msg
import time
import pyrealsense2
from cv_bridge import CvBridge, CvBridgeError
import cv2
global obj,loc,cnt,depth_img,pose,rect

obj = rospy.get_param('class','bottle')

rect = None
pose = [0,0,0]
depth_img = np.zeros((480,640))
loc = [None,None]
cnt = 0

def callback(msg):
    global obj,loc,cnt,pose,depth_img,rect
    bounding_boxes = msg.bounding_boxes
    if(cnt>0):
        for box in bounding_boxes:
            if(box.Class==obj):
                xmin = box.xmin
                ymin = box.ymin
                xmax = box.xmax
                ymax = box.ymax
                rect = [xmin,ymin,xmax,ymax]
                x = rect[2] - (rect[2]-rect[0])/2
                y = rect[3] - (rect[3]-rect[1])/2
                loc = [x,y]
            # else:
            #     loc = [None,None]
    print(obj,' loc = ',loc,' 3dPose=',pose)
    


def object_count(msg):
    global cnt
    cnt = msg.count

def convert_depth_to_phys_coord_using_realsense(x, y, depth, cameraInfo):  
    _intrinsics = pyrealsense2.intrinsics()
    _intrinsics.width = cameraInfo.width
    _intrinsics.height = cameraInfo.height
    _intrinsics.ppx = cameraInfo.K[2]
    _intrinsics.ppy = cameraInfo.K[5]
    _intrinsics.fx = cameraInfo.K[0]
    _intrinsics.fy = cameraInfo.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = pyrealsense2.distortion.none
    _intrinsics.coeffs = [i for i in cameraInfo.D]  
    result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)  
    #result[0]: right, result[1]: down, result[2]: forward
    return result

def cam_info(msg):
    global depth_img,loc,pose
    d = depth_img[loc[0]][loc[1]]
    pose = convert_depth_to_phys_coord_using_realsense(loc[0],loc[1],d,msg)


def vis_callback(msg):
    global depth_img
    track_im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    bridge = CvBridge()
    try:
        im = bridge.imgmsg_to_cv2(msg, "passthrough")
    except CvBridgeError as e:
      print(e)
    depth_img = im
    show_image(im)

def show_image(img):
    global rect,pose, obj
    img = cv2.normalize(img, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX)
    img = cv2.rectangle(img, (rect[0],rect[1]), (rect[2],rect[3]), (255, 0, 0), 2)
    text = obj+' '+str(np.around(pose,decimals=3))
    image = cv2.putText(img, text, (rect[2] - (rect[2]-rect[0])/2,rect[3] - (rect[3]-rect[1])/2), cv2.FONT_HERSHEY_SIMPLEX, 
                    0.8, (255, 0, 0), 1, cv2.LINE_AA)
    cv2.imshow("Pose Track (mm)", img)
    cv2.waitKey(3)

if __name__=='__main__':
    rospy.init_node('yolo_pixel_to_3Dpoint')
    rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback) 
    rospy.Subscriber("/darknet_ros/found_object", ObjectCount, object_count) 
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, vis_callback) 
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, cam_info) 
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        pose_tf = np.array(pose)
        br.sendTransform((pose_tf[2]/1000, -pose_tf[0]/1000, pose_tf[1]/1000),
                            (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(),
                            "object",
                            "camera_link")
    # rate.sleep()
    rospy.spin()