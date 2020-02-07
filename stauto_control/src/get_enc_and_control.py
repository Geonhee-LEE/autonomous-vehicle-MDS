#! /usr/bin/env python

import rospy
from pangyo_control.msg import ControlCommand
from std_msgs.msg import Int32, Float32

from math import *

import math
import time
import serial


port = "/dev/ttyUSB0"

S = chr(0x53)
T = chr(0x54)
X = chr(0x58)
AorM = chr(0x01)
ESTOP = chr(0x00)
GEAR = chr(0x00)
SPEED0 = chr(0x00)
SPEED1 = chr(0x00)
STEER0 = chr(0X02)
STEER1 = chr(0x02)
BRAKE = chr(0x01)
ALIVE = 0
ETX0 = chr(0x0d)
ETX1 = chr(0x0a)
Packet=[]
read=[]
count=0
count_alive=0
##################################################
def GetAorM():
    AorM = chr(0x01)
    return  AorM

def GetESTOP():
    ESTOP = chr(0x00)
    return  ESTOP

def GetGEAR(gear):
    GEAR = chr(gear)
    return  GEAR

def GetSPEED(speed):
    global count
    SPEED0 = chr(0x00)
    SPEED1 = chr(speed)
    return SPEED0, SPEED1

def GetSTEER(steer):
    steer=2*71
    steer_max=0b0000011111010000#+2000
    steer_0 = 0b0000000000000000
    steer_min=0b1111100000110000#-2000

    #print(steer)
    if (steer>=0):
        angle=int(steer)
        STEER=steer_0+angle
    else:
        angle=int(-steer)
        angle=2000-angle
        STEER=steer_min+angle

    STEER0=STEER & 0b1111111100000000
    STEER0=STEER0 >> 8
    STEER1=STEER & 0b0000000011111111

    return chr(STEER0), chr(STEER1)

def GetBRAKE(brake):
    BRAKE = chr(brake)
    return  BRAKE

def Send_to_ERP42(gear, speed, steer, brake):
    global S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1, count_alive

    count_alive = count_alive+1
    if count_alive==0xff:
        count_alive=0x00
    #print("{}, {}".format(steer,speed))

    AorM = GetAorM()
    ESTOP = GetESTOP()
    GEAR = GetGEAR(gear)
    SPEED0, SPEED1 = GetSPEED(speed)
    STEER0, STEER1 = GetSTEER(steer)
    BRAKE = GetBRAKE(brake)

    ALIVE = chr(count_alive)
    vals = [S, T, X, AorM, ESTOP, GEAR, SPEED0, SPEED1, STEER0, STEER1, BRAKE, ALIVE, ETX0, ETX1]

    for i in range(len(vals)):
        ser.write(vals[i])
    #time.sleep(0.2)
cur_ENC_backup=0
cur_steer_backup=0
def Read_from_ERP42():
    global Packet, cur_ENC_backup, cur_steer_backup

    Packet=ser.readline()





    if (len(Packet)==18):
        steer0=ord(Packet[8:9])
        steer1=ord(Packet[9:10])

        if ((steer1>=0) & (steer1<=8)):
            steer = float(-(steer1*255 + steer0)/71.)
        elif ((steer1>=248) & (steer1<=255)):
            steer = float(((255-steer1)*255 + (255-steer0))/71.)
        print(steer0, steer1, steer)
        cur_steer_backup = steer
        cur_ENC=ord(Packet[11:12])
        cur_ENC_backup=cur_ENC
        return cur_ENC, steer

    else:
        return cur_ENC_backup, cur_steer_backup


################################################################################

gear = 0
speed = 0
steer = 0
brake = 0

def cmd_callback(data):
    global gear, speed, steer, brake

    gear = data.gear
    speed = data.speed
    steer = data.steer
    brake = data.brake
    #print(steer)
    #print("{}, {}, {}. {}".format(gear,speed, steer, brake))

if __name__ == '__main__':
    rospy.init_node('serial_node')
    cmd_pub = rospy.Subscriber("CONTROL_COMMAND",ControlCommand,cmd_callback)
    pub_encoder = rospy.Publisher('ENCODER_DATA', Int32, queue_size=10)
    pub_steer = rospy.Publisher('STEER_DATA', Float32, queue_size=10)

    rate = rospy.Rate(20)

    port = rospy.get_param("~CTRL_PORT",default=port)
    #print(port)
    ser = serial.serial_for_url(port, baudrate=115200, timeout=1)


    while (ser.isOpen() and (not rospy.is_shutdown())):
        #Send_to_ERP42(gear, speed, steer, brake)
        ENC,STEER = Read_from_ERP42()
        #print(ENC, STEER)
        pub_encoder.publish(ENC)
        pub_steer.publish(STEER)
        pub_steer
