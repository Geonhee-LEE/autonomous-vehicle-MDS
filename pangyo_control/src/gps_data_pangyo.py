#! /usr/bin/env python
#import keyboard
import serial
import socket as soc
import rospy
import time
from ntrip.NtripClient import *

import multiprocessing
from multiprocessing import Process, Queue
from pangyo_control.msg import GPS

import os
import sys

import math
port = "/dev/ttyACM0"
gps_data_bef = ""


def convert_degree_to_meter(lat_data,lon_data):
    alpha = lat_data*math.pi/180
    beta = lon_data*math.pi/180

    a = 6377397.155
    f = 1/299.1528128
    b = a*(1-f)
    k = 1
    x_plus = 500000
    y_plus = 200000

    e1 = (a**2-b**2)/a**2
    e2 = (a**2-b**2)/b**2
    alpha0 = 38 *math.pi/180
    beta0 = (125+0.002890277)*math.pi/180

    T = pow(math.tan(alpha),2)
    C = e1/(1-e1)*pow(math.cos(alpha),2)
    AA = (beta-beta0)*math.cos(alpha)  #both radian

    N = a/math.sqrt( 1-e1*math.sin(alpha)**2 )
    M = a*(alpha*(1-e1/4-3*e1**2/64-5*e1**3/256)-math.sin(2*alpha)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
        +math.sin(4*alpha)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*math.sin(6*alpha)/3072)

    M0 = a*(alpha0*(1-e1/4-3*e1**2/64-5*e1**3/256)-math.sin(2*alpha0)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
         +math.sin(4*alpha0)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*math.sin(6*alpha0)/3072)

    Y = y_plus + k*N*(AA+AA**3*(1-T+C)/6+pow(AA,5)*(5-18*T+T*T+72*C-58*e2)/120)
    X = x_plus + k*(M-M0+N*math.tan(alpha)*(AA*AA/2 + pow(AA,4)*(5-T+9*C+4*C*C)/24 +
        pow(AA,6)*(61-58*T+T*T+600*C-330*e2)/720))

    return [X,Y]


class SocketInfo():
    HOST=""
    PORT=7777
    BUFSIZE=10

    def __init__(ADDRESS = "127.0.0.1"):
        HOST = ADDRESS

    ADDR=(HOST, PORT)

def cb_imu(data):
    yaw = data

fix_type={ '0' : "Invalid",
           '1' : "GPS fix (SPS)",
           '2' : "DGPS fix",
           '3' : "PPS fix",
           '4' : "Real Time Kinematic",
           '5' : "Float RTK",
           '6' : "estimated (dead reckoning) (2.3 feature)",
           '7' : "Manual input mode",
           '8' : "Simulation mode"}


if __name__ == '__main__':
    rospy.init_node("gps_node")

    #yaw_sub = rospy.Subscriber("yaw_imu",Quaternion,cb_imu)
    pos_pub = rospy.Publisher("GPS",GPS,queue_size=1)
    pos = GPS()
    port = rospy.get_param("~GPS_PORT",port)
    print(port)

    TCP_info = SocketInfo()
    TCP_sock = soc.socket(soc.AF_INET, soc.SOCK_STREAM)
    #print("main : {}".format(os.getpid()))

    ntripArgs = {}
    #ntripArgs['lat']=37.16
    #ntripArgs['lon']=127.30
    #SUWON
    ntripArgs['lat']=37.6185
    ntripArgs['lon']=127.0983
    #SOUL

    ntripArgs['height']=73.901
    ntripArgs['host']=False
    ntripArgs['ssl']=False

    ntripArgs['user']="gnss"+":"+"gnss"
    ntripArgs['caster']="gnssdata.or.kr"
    ntripArgs['port']=int("2101")

    #ntripArgs['mountpoint']="SUWN-RTCM31"
    ntripArgs['mountpoint']="SOUL-RTCM31"

    ntripArgs['V2']=True

    ntripArgs['verbose']=False
    ntripArgs['headerOutput']=None

    maxReconnect=1
    maxConnectTime=1200
    ser = serial.serial_for_url(port,115200, timeout=0)
    #multiprocessing.set_start_method('spawn', True)
    nclient = NtripClient(**ntripArgs)
    que = Queue()
    que_pos = Queue()

    proc = Process(target=nclient.update_RTK, args=(que,que_pos,))

    proc.start()
    isrunning=True
    isReady=False
    count_ready = 0
    t=time.time()
    prev_pos = [0.0,0.0]
    Line = 0.0

    while isrunning:
        RoverMessege=ser.readline().decode('ascii')

        if que.empty()==False:
            data = que.get()[0]
            
            if (type(data) is bool):
                pass
            elif (len(data)>10):
                ser.write(data)
            else:
                pass

        t = time.time()
        #print(RoverMessege)
        try:
            if "GGA" in RoverMessege:
                data=RoverMessege.split(",")

                lat = round(float(data[2]),5)
                lon = round(float(data[4]),5)

                lat_str = str(data[2]); lon_str = str(data[4])

                if(len(lat_str)==12):
                    deg_lat = int(float(lat_str))/100
                    min_lat = float(lat_str)-deg_lat*100
                    #print(deg_lat, min_lat)

                if(len(lon_str)==13):
                    deg_lon = int(float(lon_str))/100
                    min_lon = float(lon_str)-deg_lon*100
                    #print(deg_lon, min_lon)


                lat_degree = round(deg_lat + (min_lat / 60.0),7)
                lon_degree = round(deg_lon + (min_lon / 60.0),7)

                #print("{} {}".format(lat_degree,lon_degree))
                print ("Fix Type : %s  North : %.7f  East : %.7f \r"% (fix_type[data[6]],lat_degree,lon_degree))


                nclient.setPosition(lat,lon)
                #if(isReady==False): print(Line)

                pos.x = lat_degree
                pos.y = lon_degree


                    #print(pos_data[0],pos_data[1])
                    #pos.x = lon_degree
                    #pos.y = lat_degree
                pos_pub.publish(pos)

        except:
            print ("Missed" ,"\r")
        #mainloop()
