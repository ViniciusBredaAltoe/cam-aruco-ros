#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
import cv2
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
def publisher(res):
    pub = rospy.Publisher('aruco_position', Pose, queue_size=1)
    rate = rospy.Rate(60) # Hz
    p = Pose()
    if len(res[0])>0:
        p.position.x = res[0][0][0][0][0]
        p.position.y = res[0][0][0][0][1]
        #p.position.z = 1.0
        # Make sure the quaternion is valid and normalized
        #p.orientation.x = 0.0
        #p.orientation.x = 0.0
        #p.orientation.x = 0.0
        #p.orientation.w = 1.0
    else:
        p.position.x = 320
        p.position.y = 240
    pub.publish(p)
    rate.sleep()

def callback(msg):
    bridge = CvBridge()
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        res = aruco_detection(cv2_img)
        cv2_img = orientation_img(cv2_img)
        cv2.imshow('ArUco-Detect',cv2_img)
        cv2.waitKey(30)
    publisher(res)

def aruco_detection(frame):
    res = cv2.aruco.detectMarkers(frame,dictionary)

    #print(res[0],res[1],len(res[2]))
    if len(res[0])>0:
        print(res[0][0][0][0])
    return res

    
    
def orientation_img(frame):
    cv2.line(frame,(320,0),(320,480),(0,0,0),4)
    cv2.line(frame,(0,240),(640,240),(0,0,0),4)
    frame = frame[:,::-1,:]
    return frame


def imagescan():
    rospy.init_node('camera_node', anonymous=True)
    rospy.Subscriber("/image_msg", Image, callback)
    cv2.destroyAllWindows()
    rospy.spin()

if __name__ == '__main__':
    imagescan()
