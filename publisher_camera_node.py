#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

def webcam_pub():
    pub = rospy.Publisher('image_msg', Image, queue_size=1)
    rospy.init_node('publisher_camera_node', anonymous=True)
    rate = rospy.Rate(60) # 60hz

    cam = cv2.VideoCapture(0)
    bridge = CvBridge()

    if not cam.isOpened():
         sys.stdout.write("Webcam is not available")
         return -1

    while not rospy.is_shutdown():
        ret, frame = cam.read()
        #cv2.imshow('test', frame)
        #print(frame.dtype)
        #cv2.waitKey(30)
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        if ret:
            rospy.loginfo("Capturing image.")

        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        webcam_pub()
    except rospy.ROSInterruptException:
        pass
