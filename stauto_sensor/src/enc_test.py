#!/usr/bin/env python

import math
from math import sin, cos, pi
import time
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3




if __name__ == '__main__':
    rospy.init_node('test')


    enc_pub = rospy.Publisher("ENCODER_DATA", Int32, queue_size=50)
    steer_pub = rospy.Publisher("STEER", Int32, queue_size=50)
    enc=0

    prevtime=time.time()
    r = rospy.Rate(10)
while not rospy.is_shutdown():
    if(time.time()-prevtime>=0.001):
        enc=enc+1
        prevtime=time.time()
        print(enc)
    if(enc>255):
        enc=0

    # enc_pub.publish(enc)
    steer_pub.publish(30)
    r.sleep()

    # next, we'll publish the odometry message over ROS