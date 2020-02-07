#! /usr/bin/env python

import cv_bridge
import cv2
import rospy
import math
import numpy as np
from pangyo_control.msg import steer_step
from std_msgs.msg import Int32
from pangyo_control.msg import ControlCommand

#from get_gps.msg import CMD_GPS
#from main_controller.msg import lidarStaticMsg
#from get_gps.msg import CMD_GPS
#from vision.msg import Camera_msg
#from std_msgs.msg import Int32
#from lidar.msg import lidarMsgArray
#from lidar.msg import lidarMsg
import os
from time import sleep

steer = 0.0
output_speed= 45
step_gps = 0
speed_gps = 0
steer_g = 0.0
steer_c = 0.0
stop_sign = 0
parking_sign = 0
parking_count=0
encoder = 0
lidar_data = []
cluster_num=0
deep_sign=100
#lidar_static = ControlCommand()
lidar_dynamic = 0
#lidar_static_flag = lidarStaticMsg()

'''
def deep_callback(deep_data):
    global deep_sign
    deep_string = deep_data.data

    if (deep_string=='Left'):
        deep_sign=0
    elif (deep_string=='Straightleft'):
        deep_sign=1
    elif (deep_string=='Red'):
        deep_sign=2
    elif (deep_string=='Green'):
        deep_sign=3
    elif (deep_string=='Count'):
        deep_sign=4
    elif (deep_string is None):
        pass
'''
def gps_callback(gps_data):
    global steer_g, step_gps
    steer_g = gps_data.steer
    step_gps = gps_data.step

'''
def camera_callback(vision_data):
    global steer_c, stop_sign
    steer_C = vision_data.distance_pixel
    stop_sign = vision_data.stopsign
'''

def parking_callback(parking_data):
    global parking_sign
    parking_sign = parking_data.data

gear_num1=0
speed_num1=0
steer_num1=0
brake_num1=0


def parking_num1_callback(parking_node_data):
    global gear_num1, speed_num1, steer_num1, brake_num1

    gear_num1 = parking_node_data.gear
    speed_num1 = parking_node_data.speed
    steer_num1 = parking_node_data.steer
    brake_num1 = parking_node_data.brake
    #print(gear_num1, speed_num1)
'''
gear_num2=0
speed_num2=0
steer_num2=0
brake_num2=0


def parking_num2_callback(parking_node_data):
    global gear_num2, speed_num2, steer_num2, brake_num2

    gear_num2 = parking_node_data.gear
    speed_num2 = parking_node_data.speed
    steer_num2 = parking_node_data.steer
    brake_num2 = parking_node_data.brake
'''

def stop_callback(data):
    global stop_sign

    stop_sign = data.data


def serial_callback(encoder_data):
    global encoder
    encoder = encoder_data.data

'''
gear_lidar_static = 0
speed_lidar_static = 0
steer_lidar_static = 0.0
brake_lidar_static = 1

def lidar_static_callback(lidar_data):
    global lidar_static
    lidar_static = lidar_data
    lidar_static.gear = lidar_data.gear
    lidar_static.speed = lidar_data.speed
    lidar_static.steer = lidar_data.steer
    lidar_static.brake = lidar_data.brake
    # print ("steer : ", lidar_static.steer)
'''
'''
gear_lidar_dynamic = 0
speed_lidar_dynamic = 0
steer_lidar_dynamic = 0.0
brake_lidar_dynamic = 1

def lidar_static_flag_callback(lidar_flag):
    global lidar_static_flag
    lidar_static_flag.FLAG = lidar_flag.FLAG
'''

def lidar_dynamic_callback(lidar_data):
    global lidar_dynamic
    lidar_dynamic = lidar_data.data


def publish_serial(gear, speed, steer, brake):
    global pub_CC
    control_output.gear=gear
    control_output.speed=speed
    control_output.steer=steer
    control_output.brake=brake

    #pub_CC=control_output
    pub_CC.publish(control_output)


if __name__ == '__main__':
    global  control_output
    rospy.init_node('middlebridge')

    rospy.Subscriber("STEER_STEP",steer_step,gps_callback)
    rospy.Subscriber("STOPPER",Int32,stop_callback)
    #rospy.Subscriber("DEEP_SIGN",DEEP_SIGN,deep_callback)
    #rospy.Subscriber("CAMERA",Camera_msg,camera_callback)

    #rospy.Subscriber("CMD_GPS",CMD_GPS,gps_callback)
    rospy.Subscriber("PARKING",Int32,parking_callback)

    rospy.Subscriber("PARKING_NUM1", ControlCommand, parking_num1_callback)


    #rospy.Subscriber("PARKING_NUM2", ControlCommand, parking_num2_callback)

    #rospy.Subscriber("ENCODER_DATA",Int32,serial_callback)

    #rospy.Subscriber ("LIDAR_STATIC" , ControlCommand, lidar_static_callback)
    rospy.Subscriber ("LIDAR_DYNAMIC" , Int32, lidar_dynamic_callback)
    #rospy.Subscriber ("STATIC_FLAG", lidarStaticMsg, lidar_static_flag_callback)


    pub_CC = rospy.Publisher("CONTROL_COMMAND",ControlCommand,queue_size=1)

    control_output = ControlCommand()

    while not rospy.is_shutdown():
        #publish_serial(0, lidar_static.speed, lidar_static.steer, 1)
        #if (lidar_static_flag.FLAG =='finish'): publish_serial(0, 20, 0, 1)
        print ("steer_g : ", steer_g)
        print ("step_gps: ", step_gps)


        if (0<=step_gps<200):
            publish_serial(0, 120, steer_g, 1)
        '''
        if (0<=step_gps<15):
            if (parking_sign==1):
                parking_count=1

            if (parking_count==0):
                publish_serial(0, 20, steer_g, 1)

            if (parking_count==1):
                if ((gear_num1==1) & (speed_num1==1) & (steer_num1==1) & (brake_num1==1)):
                    parking_finish = 1
                    publish_serial(0, 40, steer_g, 1)
                else:
                    publish_serial(gear_num1, speed_num1, steer_num1, brake_num1)

        elif (15<=step_gps<23):
            publish_serial(0, 40, steer_g, 1)

        elif (23<=step_gps<38):
            if (parking_finish==0):
                if (parking_sign==1):
                    parking_count=1

                if (parking_count==0):
                    publish_serial(0, 20, steer_g, 1)

                if (parking_count==1):
                    if ((gear_num2==1) & (speed_num2==1) & (steer_num2==1) & (brake_num2==1)):
                        publish_serial(0, 40, steer_g, 1)
                    else:
                        publish_serial(gear_num1, speed_num1, steer_num1, brake_num1)
            else:
                publish_serial(0, 40, steer_g, 1)

        elif (38<=step_gps<94):
            publish_serial(0, 50, steer_g, 1)

        elif (94<=step_gps<107):
            publish_serial(0, 30, steer_g, 1)

        elif (107<=step_gps<117):
            if (step_gps==107):
                deep_sign=100
            if (deep_sign==100):
                publish_serial(0, 40, steer_g, 1)
            elif ((deep_sign==1) | (deep_sign==3)):
                publish_serial(0, 40, steer_g, 1)
            elif (deep_sign==2):
                if (stop_sign==0):
                    publish_serial(0, 40, steer_g, 1)
                else:
                    publish_serial(0, 0, steer_g, 100)

        elif (117<=step_gps<132):
            publish_serial(0, 50, steer_g, 1)

        elif (132<=step_gps<141):
            if (step_gps==132):
                deep_sign=100
            if (deep_sign==100):
                publish_serial(0, 40, steer_g, 1)
            elif ((deep_sign==1) | (deep_sign==3)):
                publish_serial(0, 40, steer_g, 1)
            elif (deep_sign==2):
                if (stop_sign==0):
                    publish_serial(0, 40, steer_g, 1)
                else:
                    publish_serial(0, 0, steer_g, 100)

        elif (141<=step_gps<168):
            publish_serial(0, 50, steer_g, 1)

        elif (55<=step_gps<60):
            if (len(lidar_data)>=1):
                publish_serial(0, 50, steer_lidar, 1)
            elif (lidar_sign=='finish'): publish_serial(0, 20, steer_g, 1)

        elif (60<=step_gps<65):
            publish_serial(0, 40, steer_g, 1)

        elif (65<=step_gps<70):
            if (step_gps==65):
                deep_sign=100
            if (deep_sign==100):
                publish_serial(0, 40, steer_g, 1)
            elif ((deep_sign==1) | (deep_sign==3)):
                publish_serial(0, 40, steer_g, 1)
            elif (deep_sign==2):
                if (stop_sign==0):
                    publish_serial(0, 40, steer_g, 1)
                else:
                    publish_serial(0, 0, steer_g, 100)


        elif (70<=step_gps<80):
            publish_serial(0, 40, steer_g, 1)

        elif (80<=step_gps<85):
            if (step_gps==80):
                deep_sign=100
            if (deep_sign==100):
                publish_serial(0, 40, steer_g, 1)
            elif ((deep_sign==1) | (deep_sign==3)):
                publish_serial(0, 40, steer_g, 1)
            elif (deep_sign==2):
                if (stop_sign==0):
                    publish_serial(0, 40, steer_g, 1)
                else:
                    publish_serial(0, 0, steer_g, 100)

        elif (85<=step_gps<90):
            publish_serial(0, 40, steer_g, 1)

        elif (90<=step_gps<95):
            if (step_gps==90):
                deep_sign=100
            if (deep_sign==100):
                publish_serial(0, 40, steer_g, 1)
            elif ((deep_sign==1) | (deep_sign==3)):
                publish_serial(0, 40, steer_g, 1)
            elif (deep_sign==2):
                if (stop_sign==0):
                    publish_serial(0, 40, steer_g, 1)
                else:
                    publish_serial(0, 0, steer_g, 100)

        elif (95<=step_gps<120):
            publish_serial(0, 40, steer_g, 1)

        elif (120<=step_gps<125):
            if (step_gps==120):
                deep_sign=100
            if (deep_sign==100):
                publish_serial(0, 40, steer_g, 1)
            elif ((deep_sign==1) | (deep_sign==3)):
                publish_serial(0, 40, steer_g, 1)
            elif (deep_sign==2):
                if (stop_sign==0):
                    publish_serial(0, 40, steer_g, 1)
                else:
                    publish_serial(0, 0, steer_g, 100)

        elif (125<=step_gps<130):
            publish_serial(0, 40, steer_g, 1)

        elif (130<=step_gps<135):
            if (step_gps==120):
                deep_sign=100
            if (deep_sign==100):
                publish_serial(0, 40, steer_g, 1)
            elif ((deep_sign==1) | (deep_sign==3)):
                publish_serial(0, 40, steer_g, 1)
            elif (deep_sign==2):
                if (stop_sign==0):
                    publish_serial(0, 40, steer_g, 1)
                else:
                    publish_serial(0, 0, steer_g, 100)

        elif (135<=step_gps<140):
            publish_serial(0, 40, steer_g, 1)
        '''
