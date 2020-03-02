#! /usr/bin/python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import cv2
import numpy as np

def callback(p):
    comando = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pub = Twist()
    eixox = p.position.x
    eixoy = p.position.y
    #print(eixoy, eixoy)
    rate = rospy.Rate(60) # Hz
    if eixoy > 240:
        if eixox > 320:
            pub.angular.z = 1
        else:
            pub.angular.z = 1
    elif eixoy < 240:
        if eixox > 320:
            pub.linear.x = 0.5
        else:
            pub.linear.x = -0.5
    else:
        pub.angular.z = 0
        pub.linear.x = 0

    comando.publish(pub)
    rate.sleep()


def positionscan():
    rospy.init_node('general_turtle', anonymous=True)
    rospy.Subscriber("/aruco_position", Pose, callback)
    rospy.spin()

if __name__ == '__main__':
    positionscan()
