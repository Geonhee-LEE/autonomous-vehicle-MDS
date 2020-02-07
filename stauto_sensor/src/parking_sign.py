#! /usr/bin/env python

import numpy as np
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
import time
import cv_bridge
import rospy
from std_msgs.msg import Int32

#prev_time=0

def convert_gray_scale(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def filter_region(image, vertices):
    mask = np.zeros_like(image)
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])
    return cv2.bitwise_and(image, mask)

def select_region_stopline(image):
    rows, cols = image.shape[:2]


    bottom_left1 = [cols * 0.18, rows * 1]
    top_left1 = [cols * 0.18, rows * 0.45]
    bottom_right1 = [cols * 0.82, rows * 1]
    top_right1 = [cols * 0.82, rows * 0.45]

    #bottom_left2 = [cols * 0.1, rows * 0.7]
    #top_left2 = [cols * 0.6, rows * 0.45]
    #bottom_right2 = [cols * 0.9, rows * 0.7]
    #top_right2 = [cols * 0.5, rows * 0.45]

    vertices1 = np.array([[bottom_left1, top_left1, top_right1, bottom_right1]], dtype=np.int32)
    #vertices2 = np.array([[bottom_left2, top_left2, top_right2, bottom_right2]], dtype=np.int32)
    a = filter_region(image, vertices1)
    #b = filter_region(image, vertices2)

    #vertices = cv2.bitwise_or(a,b)
    return a

def select_white(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    sensitivity_white = 60
    lower_white = np.array([0,0,255-sensitivity_white])
    upper_white = np.array([180,sensitivity_white,255])

    mask_hsv = cv2.inRange(hsv, lower_white, upper_white)


    lower_white = np.array([190-sensitivity_white,190-sensitivity_white,190-sensitivity_white])
    upper_white = np.array([255,255,255])

    mask_rgb = cv2.inRange(rgb, lower_white, upper_white)

    mask = cv2.bitwise_or(mask_rgb, mask_rgb)

    return cv2.bitwise_and(image, image, mask=mask)

###########################################################################

count=0
prevTime=0
bool_count=0
def Camera(frame):
    global count, prevTime
    #if ((time.time()-prevTime>=1) & (count<3)):
    #    count=0
    if (time.time()-prevTime>5):
        count=3

    #print(count)
    if (count<3):

        white_image = select_white(frame)

        gray_scale = convert_gray_scale(white_image)

        #WarpPerspecitve_line = WarpPerspecitve(gray_scale)
        #_, OTSUThreshholded = cv2.threshold(gray_scale, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        masked_image = select_region_stopline(gray_scale)
        Canny = cv2.Canny(masked_image, threshold1=50, threshold2=200, apertureSize=3, L2gradient=True)
        contours, hierachy = cv2.findContours(Canny, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)


        for cnt in contours:
            area = cv2.contourArea(cnt)

            epsilon = 0.005 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            size = len(approx)
            #print(size)


            if 200000>area>30000:
                if (size == 4):
                    cv2.drawContours(frame, [cnt], 0, (0, 0, 255), 2)  # blueq
                    M = cv2.moments(cnt)

                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
                    print(area,(cx,cy))

                    count = count +1
                    prevTime = time.time()

        return 0, frame



    else:

        return 1, frame

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

# Brightness : 0 to 255, Focus : 0 to 100, TH_value : 0 to 255
Brightness = 0
Focus = 0
TH_value = 180

#cap.set(cv2.CAP_PROP_AUTOFOCUS, False)
#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, False)
#cap.set(cv2.CAP_PROP_BRIGHTNESS, Brightness)
#cap.set(28, Focus)

if __name__ == '__main__':

    rospy.init_node('Parking_node', anonymous=True)

    pub_control = rospy.Publisher('PARKING', Int32, queue_size=5)

    rate = rospy.Rate(20)

    prevTime=time.time()

    while not rospy.is_shutdown():
        try:
            ret, frame = cap.read()

            Flag, image = Camera(frame)

            #cv2.imshow('frame2', image)
            print(Flag)
            pub_control.publish(Flag)
            if cv2.waitKey(1) & 0xFF == ord('w'):
                count=count+1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except rospy.ROSInterruptException:
            pass
