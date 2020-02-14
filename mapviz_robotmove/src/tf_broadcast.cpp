#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

std::string car_name;


void poseCallback(const geometry_msgs::PoseStampedConstPtr msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, msg->pose.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", car_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  if (argc != 2){ROS_ERROR("need car name as argument"); return -1;};
  car_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(car_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};