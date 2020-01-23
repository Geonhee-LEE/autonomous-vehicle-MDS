#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

tf::Quaternion yaw;
double x=0,y=0,prev_x=0,prev_y=0;
double yaw_gps=0,yaw_first=0;
tf::Quaternion gps_quat;

void yawCallback(const geometry_msgs::QuaternionConstPtr &yaw_val)
{
    yaw.setW(yaw_val->w);
    yaw.setX(yaw_val->x);
    yaw.setY(yaw_val->y);
    yaw.setZ(yaw_val->z);

    //ROS_INFO("%.4f, %.4f, %.4f, %.4f",yaw.getX(),yaw.getY(),yaw.getZ(),yaw.getW());
}

void posCallback(const geometry_msgs::PointStampedConstPtr &pos_val)
{
    x=pos_val->point.x;
    y=pos_val->point.y;

    double dx = x-prev_x;
    double dy = y-prev_y;
    double line = sqrt(dx*dx + dy*dy);
    //ROS_INFO("%.4f",line);

    if ((x != 0) && (prev_x != 0) && (line >=0.0010))
    {
        yaw_gps = atan2(dy, dx);
    }
    prev_x = x;
    prev_y = y;
    //ROS_INFO("%.4f, %.4f",x,y);
}

void yawfirstCallback(const std_msgs::Float32ConstPtr &yaw_val)
{
    yaw_first = yaw_val->data;
    if(gps_quat.getW()==0.0)
    {
        gps_quat.setEuler(yaw_first,0,0);
        //ROS_INFO("yaw_first : %.4f",yaw_first);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv,"estimate_tf");
    ros::NodeHandle nh;
    ros::Subscriber sub_yaw = nh.subscribe<geometry_msgs::Quaternion>("yaw_imu", 1, &yawCallback); 
    ros::Subscriber sub_pos = nh.subscribe<geometry_msgs::PointStamped>("pos_gps", 1, &posCallback); 
    ros::Subscriber sub_yaw_first = nh.subscribe<std_msgs::Float32>("yaw_first", 1, &yawfirstCallback); 
    ros::Publisher pub_yaw_gps = nh.advertise<std_msgs::Float32>("yaw_gps",1);
    tf::TransformBroadcaster tf_broadcaster;

    geometry_msgs::TransformStamped tf_pose;

    while (ros::ok())
    {
        if (yaw_first != 0)
        {
            tf_pose.header.stamp = ros::Time::now();
            tf_pose.header.frame_id = "base_link";
            tf_pose.child_frame_id = "pose";
            
            tf_pose.transform.rotation.x = yaw.getX();
            tf_pose.transform.rotation.y = yaw.getY();
            tf_pose.transform.rotation.z = yaw.getZ();
            tf_pose.transform.rotation.w = yaw.getW();
            
            // tf_pose.transform.rotation.x = 0;
            // tf_pose.transform.rotation.y = 0;
            // tf_pose.transform.rotation.z = 0;
            // tf_pose.transform.rotation.w = 1;
            
            // USE IMU_YAW
            
            /*
            if (yaw_gps != 0.0)
            {
                gps_quat.setEuler(yaw_gps, 0, 0);
                ROS_INFO("%.4f", yaw_gps);
            }

            tf_pose.transform.rotation.x = gps_quat.getX();
            tf_pose.transform.rotation.y = gps_quat.getY();
            tf_pose.transform.rotation.z = gps_quat.getZ();
            tf_pose.transform.rotation.w = gps_quat.getW();
            */
            tf_pose.transform.translation.x = x;
            tf_pose.transform.translation.y = y;
            tf_pose.transform.translation.z = 0.0;

            tf_broadcaster.sendTransform(tf_pose);

            std_msgs::Float32 yaw_gps_data;
            yaw_gps_data.data = yaw_gps;
            pub_yaw_gps.publish(yaw_gps_data);
        }
        ros::spinOnce();
    }
}
