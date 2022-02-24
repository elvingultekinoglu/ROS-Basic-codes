#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry odom;

void call_back_func(const nav_msgs::Odometry::ConstPtr& msg) { 
	
	ROS_INFO("%f", odom.pose.pose.position.x);
	ROS_INFO("%f", odom.pose.pose.position.y);
	ROS_INFO("%f", odom.pose.pose.position.z); 

} 

int main (int argc, char **argv) 
{ 
	ros::init(argc, argv, "konum_publisher");
	ros::NodeHandle nh;
	ros::Subscriber konum_sub=nh.subscribe("/mavros/global_position/local",1000,call_back_func);
	ros::spin();

} 
