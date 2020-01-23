#! /usr/bin/env python

import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
import time
import cv_bridge
import rospy
from pangyo_control.msg import ControlCommand
from std_msgs.msg import Int32

parking_sign=0

def parking_callback(parking_data):
    global parking_sign
    parking_sign = parking_data.data


def encode_front(ENC,cur_ENC,goal):
    if (ENC<goal):
        result=255-(goal-ENC)

    else:
        result=ENC-goal

    return result

def encode_back(ENC,cur_ENC,goal):
    if ((goal+ENC)>255):
        result=goal-(255-ENC)

    else:
        result=ENC+goal

    return result

algorithm_count=0
ENC_count=0
num_count=0
ENC=0
prevTime=0
encode_num=0

def algorithm(cur_ENC):
    global algorithm_count, ENC_count, num_count, ENC, prevTime, encode_num  # algorithm_ENC_num_count=algorithm_C , ENC_C, encode_num


    if(algorithm_count==0):
        if (ENC_count==0):
            ENC=cur_ENC
            encode_num=encode_back(ENC,cur_ENC,5)
            ENC_count=ENC_count+1
            return 2, 1,  0, 40


        elif ((cur_ENC>=encode_num+4) | (cur_ENC<=encode_num-4)):
           # print(cur_ENC,encode_num)

            return 2, 1,  0, 40

        else:
            algorithm_count=algorithm_count+1
            ENC_count=0
            prevTime = time.time()

            return 2, 1,  0, 40 ## GEAR, STEER, SPEED

    elif(algorithm_count==1):
        if (time.time()-prevTime <=1):
            return 0, 100, 0, 0
        else:
            algorithm_count=algorithm_count+1
            ENC_count=0
            return 0, 100, 0, 0

    elif(algorithm_count==2):
        if (ENC_count==0):
            ENC=cur_ENC
            encode_num=encode_front(ENC,cur_ENC,100)
            ENC_count=ENC_count+1
            return 0, 1,  28, 40

        elif ((encode_num-4>=cur_ENC) | (cur_ENC>=encode_num+4)):
            #print(cur_ENC,encode_num)
            return 0, 1, 28, 40
        else:
            algorithm_count=algorithm_count+1
            ENC_count=0
            prevTime = time.time()

            return 0, 1, 28, 40

    elif(algorithm_count==3):
        if (time.time()-prevTime <=1):
            return 0, 100, 0, 0
        else:
            algorithm_count=algorithm_count+1
            ENC_count=0
            return 0, 100, 0, 0

    elif(algorithm_count==4):
        if (ENC_count==0):
            ENC=cur_ENC
            encode_num=encode_front(ENC,cur_ENC,200)
            ENC_count=ENC_count+1
            return 0, 1, 0, 40
        elif ((encode_num-4>=cur_ENC) | (cur_ENC>=encode_num+4)):
           # print(cur_ENC,encode_num)
            return 0, 1, 0, 40
        else:
            algorithm_count=algorithm_count+1
            ENC_count=0
            prevTime = time.time()

            return 0, 1, 0, 40

    elif(algorithm_count==5):
        if (ENC_count==0):
            ENC=cur_ENC
            encode_num=encode_front(ENC,cur_ENC,110)
            ENC_count=ENC_count+1
            return 0, 1, 0, 40
        elif ((encode_num-4>=cur_ENC) | (cur_ENC>=encode_num+4)):
           # print(cur_ENC,encode_num)
            return 0, 1, 0, 40
        else:
            algorithm_count=algorithm_count+1
            ENC_count=0
            prevTime = time.time()

            return 0, 1, 0, 40


    elif(algorithm_count==6):
        if (time.time()-prevTime <=4):
            return 0, 100, 0, 0
        else:
            algorithm_count=algorithm_count+1
            ENC_count=0

            return 0, 100, 0, 0

    elif(algorithm_count==7):
        if (ENC_count==0):
            ENC=cur_ENC
            encode_num=encode_back(ENC,cur_ENC,200)
            ENC_count=ENC_count+1
            return 2, 1, 0, 40
        elif ((cur_ENC>=encode_num+4) | (cur_ENC<=encode_num-4)):
           # print(cur_ENC,encode_num)
            return 2, 1, 0, 40
        else:
            algorithm_count=algorithm_count+1
            ENC_count=0
            prevTime = time.time()

            return 2, 1, 0, 40

    elif(algorithm_count==8):
        if (ENC_count==0):
            ENC=cur_ENC
            encode_num=encode_back(ENC,cur_ENC,110)
            ENC_count=ENC_count+1
            return 2, 1, 0, 40
        elif ((cur_ENC>=encode_num+4) | (cur_ENC<=encode_num-4)):
           # print(cur_ENC,encode_num)
            return 2, 1, 0, 40
        else:
            algorithm_count=algorithm_count+1
            ENC_count=0
            prevTime = time.time()

            return 2, 1, 0, 40

    elif(algorithm_count==9):
        if (time.time()-prevTime <=1):
            return 0, 100, 0, 0
        else:
            algorithm_count=algorithm_count+1
            ENC_count=0
            prevTime = time.time()
            return 0, 100, 0, 0

    elif(algorithm_count==10):
        if (ENC_count==0):
            ENC=cur_ENC
            encode_num=encode_back(ENC,cur_ENC,100)
            ENC_count=ENC_count+1
            return 2, 1, 28, 40
        elif ((cur_ENC>=encode_num+4) | (cur_ENC<=encode_num-4)):
           # print(cur_ENC,encode_num)
            return 2, 1, 28, 40
        else:

            algorithm_count=algorithm_count+1
            ENC_count=0

            return 2, 1, 28, 40

    else:
        return 1, 1, 1, 1



cur_ENC=0
def callback(data):
    global cur_ENC
    cur_ENC=data.data
   # print(cur_ENC)



if __name__ == '__main__':

    rospy.init_node('Parking_node_num1', anonymous=True)

    rospy.Subscriber("ENCODER_DATA", Int32, callback)

    rospy.Subscriber("PARKING",Int32,parking_callback)

    pub_control = rospy.Publisher('PARKING_NUM1', ControlCommand, queue_size=5)

    rate = rospy.Rate(10)
    output = ControlCommand()



    while not rospy.is_shutdown():
        try:
            if (parking_sign==1):

                GEAR, BRAKE, STEER, SPEED = algorithm(cur_ENC)
                print(GEAR, SPEED, STEER, BRAKE)
            #cv2.imshow('frame2', lane_detection)
                output.gear = GEAR
                output.speed = SPEED
                output.steer=STEER
                output.brake=BRAKE

                pub_control.publish(output)

            else:
                output.gear = 0
                output.speed = 0
                output.steer= 0
                output.brake= 0
                print(0, 0, 0, 0)

                pub_control.publish(output)

        except rospy.ROSInterruptException:
            pass
