#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def enc_callback(data):
    global curl_encoder, current_time

    curl_encoder=data.data

def steer_callback(data):
    global steer

    steer=data.data
    steer += 1.7

def calculation(steer,dt):
    global curl_encoder, last_encoder
    
    if(curl_encoder==last_encoder):
        speed=0
    if(curl_encoder > last_encoder):
        try:
            print(curl_encoder)
            print(last_encoder)
            move_encoder = abs(curl_encoder-last_encoder -255)
            last_encoder = curl_encoder
            rad = move_encoder*0.06283185307 # 3.6[deg] * 3.14159265359 / 180 = 0.06283185307, encoder 100 -> 1 rev
            wheel_vth = rad/dt
            speed = wheel_vth * 0.265 #wheel_radius
            print(move_encoder, speed)

        except ZeroDivisionError:
            speed=0


    elif(curl_encoder < last_encoder):
        try:
            move_encoder = abs(curl_encoder-last_encoder)
            # print(move_encoder)
            last_encoder = curl_encoder
            rad = move_encoder*0.06283185307 # 3.6[deg] * 3.14159265359 / 180 = 0.06283185307, encoder 100 -> 1 rev

            wheel_vth = rad/dt
            speed = wheel_vth * 0.265 #wheel_radius

        except ZeroDivisionError:
            speed=0

    else:
        move_encoder = 0

        speed = 0

    #print(speed)
    vx = speed
    vy = 0
    #print(steer, speed)
    steer = steer*(pi/180)
    #print(math.tan((pi/2)+steer))
    #print(steer)

    if(steer < 0.):
        try:
            r = 1.03*math.tan((pi/2)+steer)
            vth = speed/r

            return vx, vy, vth
        except ZeroDivisionError:
            vth = 0

            return vx, vy, vth


    elif(steer > 0.):
        try:
            r = -1.03*math.tan((pi/2)-steer)
            vth = speed/r
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

    rospy.Subscriber("encoder_data",Int32,enc_callback)
    rospy.Subscriber("steer_data",Float32,steer_callback)

    odom_pub = rospy.Publisher("encoder_odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()


    curl_encoder=0
    steer=0
    last_encoder=0

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0
    vy = 0
    vth = 0

    dt = 0

    last_encoder = curl_encoder


    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(10)


    while not rospy.is_shutdown():


        # compute odometry in a typical way given the velocities of the robot

        #last_time = current_time
        #rospy.spinOnce()
        current_time = rospy.Time.now()

        dt = (current_time - last_time).to_sec() + (current_time - last_time).to_nsec()*1e-9
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
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

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

        last_time = current_time
        r.sleep()
