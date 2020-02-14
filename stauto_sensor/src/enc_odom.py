#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Float32, Float64, Header
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped

cur_encoder = 0     # Encoder counting
prev_encoder = 0
steer = 0           # -2000 ~ 2000 (actual steering angle * 71)
speed = 0           # 0~200(actual speed(KPH)*10) 

class initpose: #initial pose set 1번밖에 못쓰는 오류 존재
    def __init__(self):
        self.pre_position = [0,0,0]
        self.position = [0,0,0]
        self.orientation_quaternion = [0,0,0,0]

        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,self.initialCallback)

    def initialCallback(self,data):
        current_time = rospy.Time.now()
        pose_stemp = data
        self.position = [pose_stemp.pose.pose.position.x, pose_stemp.pose.pose.position.y, pose_stemp.pose.pose.position.z]
        self.orientation_quaternion = [pose_stemp.pose.pose.orientation.x, pose_stemp.pose.pose.orientation.y, pose_stemp.pose.pose.orientation.z, pose_stemp.pose.pose.orientation.w]

        odom_broadcaster.sendTransform(
            self.position,
            self.orientation_quaternion,
            current_time,
            "odom",
            "map"
        )
        print("init")

def EncCallback(data):
    global cur_encoder, current_time

    cur_encoder=data.data

def SteerCallback(data):
    global steer

    steer= (data.data) * 0.014

def SpeedCallback(data):
    global erp_speed

    erp_speed = data.data/72


########### odometry calculation ###############

def calculation(steer,dt,erp_speed_):
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
            if(diff_encoder > 100000): # first error check
                speed = 0
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
    steer = steer*(pi/180)


    if(steer < 0.):
        try:
            r = 1.03/math.tan(steer)
            vth = speed* 0.82/r #constant is parameter(weight)

            return vx, vy, vth
        except ZeroDivisionError:
            vth = 0

            return vx, vy, vth


    elif(steer > 0.):
        try:
            r = 1.03/math.tan(steer)
            vth = speed* 0.3/r #constant is parameter(weight)

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

    class_init_pose = initpose() # initial pose

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    erp_speed = 0
    cur_encoder=0
    steer=0
    prev_encoder=0

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0
    vy = 0
    vth = 0

    dt = 0

    prev_encoder = cur_encoder


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
            vx, vy, vth = calculation(steer,dt,erp_speed)
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th

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

        last_time = current_time
        r.sleep()
