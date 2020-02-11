#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <std_msgs/Empty.h>

//serial::Serial ser;

char S = 0x53;
char T = 0x54;
char X = 0x58;
char AorM = 0x01;
char ESTOP = 0x00;
char GEAR = 0x00;
char SPEED0 = 0x00;
char SPEED1 = 0x00;
char STEER0 = 0x00;
char STEER1 = 0x00;
char BRAKE = 0x00;
char ENC0 = 0x00;
char ENC1 = 0x00;
char ENC2 = 0x00;
char ENC3 = 0x00;
char ALIVE = 0x00;
char ETX0 = 0x0d;
char ETX1 = 0x0a;



int main(int argc, char **argv)
{
    ros::init(argc,argv,"ERP42_decoder");
    ros::NodeHandle n;

    ros::Publisher pub_encoder_erp42 = n.advertise<std_msgs::Float32>("ERP42_encoder",10);
    ros::Publisher pub_steer_erp42 = n.advertise<std_msgs::Float32>("ERP42_steer",10);
    ros::Publisher pub_speed_erp42 = n.advertise<std_msgs::Float32>("ERP42_speed",10);
    ros::Publisher pub_gear_erp42 = n.advertise<std_msgs::Float32>("ERP42_gear",10);
    ros::Publisher pub_brake_erp42 = n.advertise<std_msgs::Float32>("ERP42_brake",10);

    ros::Rate loop_rate(50);

    /*
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();

    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("unable to open port");
        return -1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port Initialized");
    }
    else { return -1;}

    while(ros::ok())
    {
        ros::spinOnce();

        if(ser.available())
        {
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String ERP42Info;
            ERP42Info.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read : " << ERP42Info.data);

        }
        loop_rate.sleep();
    }
    */
}