#! /usr/bin/env python
import numpy as np
import rospy
import pynput
import time
import rospkg
from math import *
from std_msgs.msg import Float32
from pangyo_control.msg import GPS
from pangyo_control.msg import steer_step

curl_gps = [0.0,0.0]
yaw = 0
step_gps=0
n_1_value = 0
error_angle=0
error_angle_1=0

isReady = False
isSafeQuit = False

find_step_gps_sign=False
find_step_gps_count=0
gps_x_avg=0
gps_y_avg=0

def convert_degree_to_meter(lat,lon):

    alpha = lat*pi/180
    beta = lon*pi/180

    a = 6377397.155
    f = 1/299.1528128
    b = a*(1-f)
    k = 1
    x_plus = 500000
    y_plus = 200000

    e1 = (a**2-b**2)/a**2
    e2 = (a**2-b**2)/b**2
    alpha0 = 38 *pi/180
    beta0 = (125+0.002890277)*pi/180

    T = pow(tan(alpha),2)
    C = e1/(1-e1)*pow(cos(alpha),2)
    AA = (beta-beta0)*cos(alpha)  #both radian

    N = a/sqrt( 1-e1*sin(alpha)**2 )
    M = a*(alpha*(1-e1/4-3*e1**2/64-5*e1**3/256)-sin(2*alpha)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
        +sin(4*alpha)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*sin(6*alpha)/3072)

    M0 = a*(alpha0*(1-e1/4-3*e1**2/64-5*e1**3/256)-sin(2*alpha0)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
         +sin(4*alpha0)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*sin(6*alpha0)/3072)

    Y = y_plus + k*N*(AA+AA**3*(1-T+C)/6+pow(AA,5)*(5-18*T+T*T+72*C-58*e2)/120)
    X = x_plus + k*(M-M0+N*tan(alpha)*(AA*AA/2 + pow(AA,4)*(5-T+9*C+4*C*C)/24 +
        pow(AA,6)*(61-58*T+T*T+600*C-330*e2)/720))

    return [X,Y]

def cb_pos(position):
    global curl_gps
    curl_gps[0] = position.x
    curl_gps[1] = position.y
    #if(offset_mode and isCal):
    #    trans[0] = trans[0] + offset_X
    #    trans[1] = trans[1] + offset_Y

def cb_yaw(yaw_value):
    global yaw
    yaw = yaw_value.data*180/3.141592

    #print(yaw)
    #print(yaw)

def find_step_gps(gps_x_avg, gps_y_avg,gps_data):
    min_step=0
    distance=2 #meter

    curl_gps_meter=convert_degree_to_meter(gps_x_avg,gps_y_avg)
    #print(curl_gps_meter)
    for i in range(len(gps_data)):

        step_gps_meter=convert_degree_to_meter(float(gps_data[i].split()[0]),float(gps_data[i].split()[1]))
        #print(step_gps_meter)
        find_meter=sqrt((curl_gps_meter[0]-step_gps_meter[0])**2+(curl_gps_meter[1]-step_gps_meter[1])**2)
        #print(find_meter)
        if (find_meter<distance):
            min_step=i

    return min_step



if __name__ == '__main__':

    rospy.init_node('controller_gps',disable_signals=True)
    #listener = tf.TransformListener()
    sub_pos = rospy.Subscriber("GPS", GPS, cb_pos)
    sub_yaw = rospy.Subscriber("yaw_degree", Float32, cb_yaw)
    pub_control = rospy.Publisher("STEER_STEP", steer_step, queue_size=5)

    rate = rospy.Rate(10)
    cmd_val = steer_step()
    rospack = rospkg.RosPack()
    rospack.list()
    path = rospack.get_path('pangyo_control') + "/gps_data/"
    file_name = "gps_data_middle.txt"
    f = open(path + file_name ,"r")

    gps_data = f.read().splitlines()

    last_step = len(gps_data)

    error_yaw=-16

    while not rospy.is_shutdown():
        try:

            if(find_step_gps_sign==False):
                if(find_step_gps_count==0):
                    time.sleep(3)
                elif(find_step_gps_count<=10):
                    gps_x_avg=gps_x_avg+curl_gps[0]
                    gps_y_avg=gps_y_avg+curl_gps[1]
                    print(gps_x_avg,gps_y_avg)
                elif(find_step_gps_count==11):
                    gps_x_avg=gps_x_avg/10
                    gps_y_avg=gps_y_avg/10
                    print(gps_x_avg,gps_y_avg)
                    step_gps=find_step_gps(gps_x_avg, gps_y_avg,gps_data)

                find_step_gps_count=find_step_gps_count+1
                #print(find_step_gps_count)
                if(find_step_gps_count>=12):
                    find_step_gps_sign=True


            else:
                print(step_gps)
                '''
                #print(curl_gps[0], curl_gps[1], yaw)
                gps0=[float(gps_data[step_gps+1].split()[0]),float(gps_data[step_gps+1].split()[1])]
                gps1=[float(gps_data[step_gps+2].split()[0]),float(gps_data[step_gps+2].split()[1])]
                gps2=[float(gps_data[step_gps+3].split()[0]),float(gps_data[step_gps+3].split()[1])]
                gps3=[float(gps_data[step_gps+4].split()[0]),float(gps_data[step_gps+4].split()[1])]
                gps4=[float(gps_data[step_gps+5].split()[0]),float(gps_data[step_gps+5].split()[1])]
                #print(gps0, gps1, gps2, gps3, gps4)
                line_data_x=[gps0[0],gps1[0],gps2[0],gps3[0],gps4[0]]
                line_data_y=[gps0[1],gps1[1],gps2[1],gps3[1],gps4[1]]

                fp1 = np.polyfit(line_data_x,line_data_y,5)
                goal_x=gps2[0]
                goal_y= fp1[0]*goal_x**(2)+fp1[1]*goal_x+fp1[2]
                '''
                goal_x=float(gps_data[step_gps].split()[0])
                goal_y=float(gps_data[step_gps].split()[1])
                if(curl_gps[0]!=0.0):
                    try:
                        gps_n = convert_degree_to_meter(curl_gps[0],curl_gps[1])
                        gps_n_1 = convert_degree_to_meter(goal_x,goal_y)


                        Line = sqrt((gps_n_1[0]-gps_n[0])**(2) + (gps_n_1[1]-gps_n[1])**(2))

                        #1 -> 1 meter
                        '''
                        angle_cur_n = atan2(gps2[1]-gps0[1], gps2[0]-gps0[0])*180/3.141592
                        '''
                        angle_cur_n = atan2(float(gps_data[step_gps+1].split()[1])-float(gps_data[step_gps].split()[1]), float(gps_data[step_gps+1].split()[0])-float(gps_data[step_gps].split()[0]))*180/3.141592






                        angle_cur_n_1 = atan2(float(gps_data[step_gps+1].split()[1])-curl_gps[1], float(gps_data[step_gps+1].split()[0])-curl_gps[0])*180/3.141592
                        #deangle_cur_n = atan2(float(gps_data[step_gps+1].split()[1])-float(gps_data[step_gps].split()[1]), float(gps_data[step_gps+1].split()[0])-float(gps_data[step_gps].split()[0]))*180/3.141592



                        #angle_cur_n=angle_cur_n-angle_cur_n_1*0.01
                        #print(angle_cur_n, angle_cur_n_1)


                        if(abs(Line) <= 2)and(step_gps<=last_step-3) :
                            step_gps=step_gps+1


                        #yaw_error=error_angle
                        #current gps : trans[0],[1] / goal : gps_n
                        sub_yaw=yaw+error_yaw
                        if ((angle_cur_n<0) & (sub_yaw<0)):
                            error_angle=angle_cur_n-sub_yaw
                        elif ((angle_cur_n<0) & (sub_yaw>=0)):
                            error_angle=angle_cur_n-sub_yaw
                        elif ((angle_cur_n>=0) & (sub_yaw>=0)):
                            error_angle=angle_cur_n-sub_yaw
                        elif ((angle_cur_n>=0) & (sub_yaw<0)):
                            error_angle=angle_cur_n-sub_yaw

                        if ((angle_cur_n_1<0) & (sub_yaw<0)):
                            error_angle_1=angle_cur_n_1-sub_yaw
                        elif ((angle_cur_n_1<0) & (sub_yaw>=0)):
                            error_angle_1=angle_cur_n_1-sub_yaw
                        elif ((angle_cur_n_1>=0) & (sub_yaw>=0)):
                            error_angle_1=angle_cur_n_1-sub_yaw
                        elif ((angle_cur_n_1>=0) & (sub_yaw<0)):
                            error_angle_1=angle_cur_n_1-sub_yaw
                        # error_angle = 0.0
                        # error_angle_n_1 = 0.0
                        error_angle_1=error_angle_1-16

                        error_angle=error_angle*0.3+error_angle_1*0.7

                        if(error_angle <= 200) : isReady = True

                        if(isReady == True):
                            '''
                            now =  time.time()
                            if((now != 0) and (prev_time != 0)):
                                dt = now - prev_time
                            prev_time = now

                            if(error_angle <= 0.12) : P_gain_n_1_new = P_gain_n_1 * 0.8
                            else : P_gain_n_1_new = P_gain_n_1

                            P_gain_n_1_new = P_gain_n_1

                            P_value = P_gain * error_angle
                            P_value_n_1 = P_gain_n_1_new * error_angle_n_1

                            if(dt != 0.0):
                                I_value = I_value - I_gain * error_angle * dt
                                I_value_n_1 = I_value_n_1 - I_gain_n_1 * error_angle_n_1 * dt

                                if(error_angle_prev != 0.0):
                                    de = error_angle - error_angle_prev
                                    de_n_1 = error_angle_n_1 - error_angle_n_1_prev

                                error_angle_prev = error_angle
                                error_angle_n_1_prev = error_angle_n_1
                                or_angle, delta
                                D_value = D_gain * de / dt
                                D_value_n_1 = D_gain_n_1 * de_n_1 / dt


                            PID_n_1 = (P_value_n_1) + (I_value_n_1) + (D_value_n_1)


                            cmd_val.steer_G = ((P_value) + (I_value) + (D_value)) / 71
                            if(n_1_mode == True) : cmd_val.steer_G = cmd_val.steer_G + (PID_n_1 / 71)
                            # PID Control
                            '''
                            #ld=Line
                            ld=4.5
                            l=1 #edit
                            '''
                            if(error_angle<=0):
                                delta=atan2(2.0*l*sin(-error_angle*3.141592/180),ld)*180/3.141592
                            else:
                                delta=atan2(2.0*l*sin(-error_angle*3.141592/180),ld)*180/3.141592

                            delta=atan2(2.0*l*sin(-error_angle*3.141592/180),ld)*180/3.141592
                            if (delta>28):
                                delta=28
                            elif (delta<-28):
                                delta=-28
                            '''
                            delta=atan2(2.0*l*sin(error_angle*3.141592/180),ld)*180/3.141592
                            p=error_angle*2
                            if (p>28):
                                p=28
                            elif (p<-28):
                                p=-28
                            if (delta>28):
                                delta=28
                            elif (delta<-28):
                                delta=-28


                            print(step_gps,Line, yaw+error_yaw, angle_cur_n, error_angle, delta)
                            cmd_val.steer = delta
                            cmd_val.step = step_gps

                            #cmd_pub.publish(cmd_val)
                            ############################################


                            pub_control.publish(cmd_val)
                    except rospy.ROSInterruptException:
                        print('error')
                        continue

        except (KeyboardInterrupt, RuntimeError) as ident:
            print('error')
