#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>  

geometry_msgs::PoseStamped pose;

int main(int argc, char** argv){

    ros::init(argc, argv, "org_code");
    ros::NodeHandle nh; 

    ros::Publisher pose_pub=nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);

    while (ros::ok()){
        
    }
} 