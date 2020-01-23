#! /usr/bin/env python

import numpy as np
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv_bridge
import rospy
from pangyo_control.msg import figure
#from pangyo_control.msg import figure_array

rectangle=[]
circle=[]


def convert_hls(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def convert_gray_scale(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


def apply_smoothing(image, kernel_size=7):
    return cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)


def detect_edges(image, low_threshold=20, high_threshold=200):
    return cv2.Canny(image, low_threshold, high_threshold)

def select_green_red(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
##############################################################################
    yellow = 20

    sensitivity_yellow = 40
    lower_yellow = np.array([yellow - sensitivity_yellow,80,80])
    upper_yellow = np.array([yellow + sensitivity_yellow,255,255])

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

##############################################################################
    # mask-white
    sensitivity_white = 20
    lower_white = np.array([0,0,255-sensitivity_white])
    upper_white = np.array([180,sensitivity_white,255])

    mask_w_hsv = cv2.inRange(hsv, lower_white, upper_white)



    lower_white = np.array([150,150,150])
    upper_white = np.array([255,255,255])

    mask_w_rgb = cv2.inRange(rgb, lower_white, upper_white)

    mask_w = cv2.bitwise_or(mask_w_hsv, mask_w_rgb)

##############################################################################
    # mask-red
    sensitivity_red = 50

    lower_red = np.array([180 - sensitivity_red, 100, 100])
    upper_red = np.array([180, 255, 255])

    mask_red = cv2.inRange(hsv, lower_red, upper_red)



##############################################################################

    green = 70
    sensitivity_green = 30

    lower_green = np.array([green - sensitivity_green,0,0])

    upper_green = np.array([green + sensitivity_green,255,255])

    mask_green = cv2.inRange(hsv, lower_green, upper_green)



    res_green = cv2.bitwise_and(image,image, mask = mask_green)

    res_red = cv2.bitwise_and(image,image, mask = mask_red)


    res_gr = cv2.bitwise_or(res_green,res_red)

    # combine the mask
    #mask_yw = cv2.bitwise_or(mask_yellow,mask_w)
    #mask_r = cv2.bitwise_or(mask_red_0, mask_red_1)

    #mask = cv2.bitwise_or(mask_yw, mask_r)

    return res_green, res_red, res_gr


def filter_region(image, vertices):
    mask = np.zeros_like(image)
    if len(mask.shape) == 2:
        cv2.fillPoly(mask, vertices, 255)
    else:
        cv2.fillPoly(mask, vertices, (255,) * mask.shape[2])
    return cv2.bitwise_and(image, mask)


def contour_rectangle(frame, Canny):
    global rectangle
    contours, hierachy = cv2.findContours(Canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    for cnt in contours:
            area = cv2.contourArea(cnt)

            epsilon = 0.03 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            size = len(approx)
            #print(size)


            if 20000>area>2000:
                if (size==4):

                    cv2.drawContours(frame, contours, -1, (0, 0, 255), 2)  # blue
                    M = cv2.moments(cnt)

                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)


                    center=[cx,cy]

                    rectangle.append(center)
                    #rectangle_count=rectangle_count+1

                    #print('rectangle', 'Area: ', area, 'Center: ',center)
    rectangle=sorted(rectangle)
    #print('rectangle: ', rectangle)
    return frame, rectangle


def contour_circle(frame, Canny):
    global circle
    contours, hierachy = cv2.findContours(Canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)


    for cnt in contours:
            area = cv2.contourArea(cnt)

            epsilon = 0.03 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            size = len(approx)
            #print(size)


            if 20000>area>1600:
                if (8<=size<=8):
                    cv2.drawContours(frame, contours, -3, (0, 255, 0), 2)  # blue
                    M = cv2.moments(cnt)

                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)


                    center=[cx,cy]
                    circle.append(center)

                    #circle[circle_count]=center
                    #circle_count=circle_count+1
                    #print('circle', 'Area: ', area, 'Center: ',center)

    circle=sorted(circle)
    #print('circle: ', circle)
    return frame, circle

def Camera(frame):

    green, red, red_green = select_green_red(frame)

    gray_scale = convert_gray_scale(red)

    _, OTSUThreshholded = cv2.threshold(gray_scale, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    ret, img_binary = cv2.threshold(OTSUThreshholded, 50, 255, 0)

    blurred_image = apply_smoothing(img_binary)

    rectangle_frame, find_rectangle = contour_rectangle(frame, blurred_image)

    circle_frame, find_circle = contour_circle(frame, blurred_image)



    #print(contours)

    #print(rectangle_frame, rectangle_center)


    return rectangle_frame, circle_frame, find_rectangle, find_circle

cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

Brightness = 0
Focus = 0
TH_value = 180

cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 35)
#cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
#cap.set(28, Focus)


if __name__ == '__main__':
    rospy.init_node('find_figure', anonymous=True)


    output_rectangle=figure()
    output_circle=figure()



    pub_rectangle = rospy.Publisher('RECTANGLE',figure, queue_size=5)
    pub_circle = rospy.Publisher('CIRCLE',figure, queue_size=5)



    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            ret, frame = cap.read()

            rectangle_frame, circle_frame, find_rectangle, find_circle=Camera(frame)
            #print(len(find_circle))
            #print(len(find_rectangle))
            if (len(find_rectangle)==0):
                pass
            else:

                output_rectangle.figure=find_rectangle[0]

            if (len(find_circle)==0):
                pass
            else:

                output_circle.figure=find_circle[0]


            #if (len(find_circle)==0):
            #    pass
            #else:
            #    for circle_count in range(0,len(find_circle)):
            #        output_circle.figure_array.append(find_circle[circle_count])



            print(output_rectangle.figure)
            pub_rectangle.publish(output_rectangle)
            pub_circle.publish(output_circle)
            #pub_circle.publish(output2_circle)
            #output_rectangle.figure_array=[]
            #output_circle.figure_array=[]
            rectangle=[]
            circle=[]

            cv2.imshow('red: rectangle / green: circle', rectangle_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except rospy.ROSInterruptException:
            pass
