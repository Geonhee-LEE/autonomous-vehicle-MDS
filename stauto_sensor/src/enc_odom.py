#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32, Float64, Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
 
cur_encoder = 0     # Encoder counting
prev_encoder = 0
steer = 0           # -2000 ~ 2000 (actual steering angle * 71)
speed = 0           # 0~200(actual speed(KPH)*10)

def EncCallback(data):
    global cur_encoder, current_time

    cur_encoder=data.data

def SteerCallback(data):
    global steer

    steer = data.data


def SpeedCallback(data):
    global speed

    speed = data.data


########### odometry calculation ###############

def calculation(steer,dt):
    global cur_encoder, prev_encoder
    
    if(cur_encoder == prev_encoder):
        speed=0

    elif(cur_encoder > prev_encoder):       # going backward
        try:
            diff_encoder = abs(cur_encoder-prev_encoder)
            prev_encoder = cur_encoder
            rad = diff_encoder*0.06283185307 # 3.6[deg] * 3.14159265359 / 180 = 0.06283185307, encoder 100 -> 1 rev

            wheel_vth = rad/dt
            speed = wheel_vth * 0.265 #wheel_radius
            print(diff_encoder, speed)

        except ZeroDivisionError:
            speed=0

    elif(cur_encoder < prev_encoder):       #going forward
        try:
            diff_encoder = abs(cur_encoder-prev_encoder)
            prev_encoder = cur_encoder
            rad = diff_encoder*0.06283185307 # 3.6[deg] * 3.14159265359 / 180 = 0.06283185307, encoder 100 -> 1 rev

            wheel_vth = rad/dt
            speed = wheel_vth * 0.265 #wheel_radius

        except ZeroDivisionError:
            speed=0


    vx = speed
    vy = 0
    steer = (steer/71)*(pi/180)


    if(steer < 0.):
        try:
            r = 1.03/math.tan(steer)
            vth = speed* 0.5454545454/r

            return vx, vy, vth
        except ZeroDivisionError:
            vth = 0

            return vx, vy, vth


    elif(steer > 0.):
        try:
            r = -1.03/math.tan(steer)
            vth = speed* 0.72/r
            # print(r)
            return vx, vy, vth
        except ZeroDivisionError:
            vth = 0

            return vx, vy, vth

    else:
        vth = 0

        return vx, vy, vth



if __name__ == '__main__':
    rospy.init_node('odometry_publisher')

    rospy.Subscriber("ERP42_encoder",Float64,EncCallback) # 4 bytes
    rospy.Subscriber("ERP42_steer",Float32,SteerCallback) # 2 bytes
    rospy.Subscriber("ERP42_speed",Float32,SpeedCallback) # 2 bytes

    odom_pub = rospy.Publisher("encoder_odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()


    x = 0.0
    y = 0.0
    th = 0.0    # yaw

    vx = 0      # x direction velocity
    vy = 0      # y direction velocity
    vth = 0     # angular velocity
    
    prev_encoder = cur_encoder

    dt = 0      # the time difference

    current_time = rospy.Time.now() 
    prev_time = rospy.Time.now()

    r = rospy.Rate(10)


    while not rospy.is_shutdown():


        # compute odometry in a typical way given the velocities of the robot

        #prev_time = current_time
        #rospy.spinOnce()
        current_time = rospy.Time.now()

        dt = (current_time - prev_time).to_sec() + (current_time - prev_time).to_nsec()*1e-9
        if dt>0:
            vx, vy, vth = calculation(steer,dt)
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th
        #print(th)
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        # odom_broadcaster.sendTransform(
        #     (x, y, 0.),
        #     odom_quat,
        #     current_time,
        #     "base_footprint",
        #     "odom"
        # )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)
        #print(odom.pose.pose)

        prev_time = current_time

        r.sleep()
