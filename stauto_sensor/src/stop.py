#! /usr/bin/env python

import cv2
import rospy
import numpy as np
import time
from std_msgs.msg import Int32

cap = cv2.VideoCapture(1)

#cap.set(cv2.CAP_MSMF,4000)
#cap = cv2.VideoCapture("Python_images/Line5.mp4")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Brightness : 0 to 255, Focus : 0 to 100, TH_value : 0 to 255
Brightness = 0
Focus = 0
TH_value = 180

cap.set(cv2.CAP_PROP_AUTOFOCUS, False)
#cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, False)
#cap.set(cv2.CAP_PROP_BRIGHTNESS, Brightness)
cap.set(28, Focus)



def convert_gray_scale(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)


def slice_image(image):
    # src = cv2.imread(image,cv2.IMREAD_COLOR)
    cv2.imshow("image", image)
    # dst = src.copy()
    slicedimage = image[680:700, 400:1000]
    cv2.imshow("slicedimage",slicedimage)

    pts1_l = np.float32([[0, 0], [600, 0], [0, 20], [600, 20]])
    pts2_l = np.float32([[0, 0], [600, 0], [0, 500], [600, 500]])
    matrix = cv2.getPerspectiveTransform(pts1_l, pts2_l)
    result = cv2.warpPerspective(slicedimage, matrix, (600, 500))

    return result


def select_white(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    sensitivity_white = 50
    lower_white = np.array([0, 0, 255 - sensitivity_white])
    upper_white = np.array([180, sensitivity_white, 255])

    mask_hsv = cv2.inRange(hsv, lower_white, upper_white)

    lower_white = np.array([150, 150, 150])
    upper_white = np.array([255, 255, 255])

    mask_rgb = cv2.inRange(rgb, lower_white, upper_white)

    mask = cv2.bitwise_or(mask_hsv, mask_rgb)

    return cv2.bitwise_and(image, image, mask=mask)


def adaptive_thresholding(image):
    ret, img = cv2.threshold(image, TH_value, 255, cv2.THRESH_BINARY)
    # ret, img = cv2.threshold(image,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # value = (1,220),(2,180),(3,140),(4,180),(5,180)
    return img


def get_pixel_value(thresholding):
    #sumx=thresholding.sum(1)
    #print (sum)
    dot_np = np.copy(thresholding)
    dotindex = np.where(dot_np == 255)
    dotindex_x = dotindex[1]
    length = len(dotindex_x)


    if length>220000:
        print (length, '1')
        return 1
    else:
        print (length, '0')
        return 0



    # 1593pixel,153reigon,1-10pixel


# 384 896

def Camera(frame):
    slicedimage = slice_image(frame)

    white_image = select_white(slicedimage)
    # cv2.imshow("white_image", white_image)
    # masked_image = select_region(white_yellow_image)

    gray_scale = convert_gray_scale(white_image)

    thresholding = adaptive_thresholding(gray_scale)

    sign = get_pixel_value(thresholding)

    return sign



if __name__ == '__main__':
    rospy.init_node('Stop_node', anonymous=True)

    pub_control = rospy.Publisher('STOPPER', Int32, queue_size=5)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:

            ret, frame = cap.read()

            Flag = Camera(frame)

            pub_control.publish(Flag)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


        except rospy.ROSInterruptException:
            pass
