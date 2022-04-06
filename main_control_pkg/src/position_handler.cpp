#include <ros/ros.h>
#include <main_control_pkg/takeoff_serv.h> 
#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Odometry.h>
#include <vector>

struct position{
    double x;
    double y;
    double z;
};

geometry_msgs::PoseStamped pose_c;
main_control_pkg::takeoff_serv srv; 
nav_msgs::Odometry odom; 

std::vector<position> positions; 

void odom_cbb(const nav_msgs::Odometry::ConstPtr &msg){
    odom= *msg;
}

#define radius 0.5

ros::ServiceClient pos_client;

void gotoposition(double x, double y, double z){
    
    srv.request.poseStamped.pose.position.x=x; 
    srv.request.poseStamped.pose.position.y=y; 
    srv.request.poseStamped.pose.position.z=z; 

    ros::Rate looprate(20); 

    while (ros::ok&&!((x-radius<odom.pose.pose.position.x&&odom.pose.pose.position.x>x+radius) &&
    (y-radius<odom.pose.pose.position.y && odom.pose.pose.position.y>y+radius) && 
    (z-radius<odom.pose.pose.position.z && odom.pose.pose.position.z>z+radius))){
        ros::spinOnce();
        ROS_INFO("%f , %f , %f" , odom.pose.pose.position.x , odom.pose.pose.position.y, odom.pose.pose.position.z); 
        looprate.sleep(); 
    }
    ros::Duration(0.5).sleep(); 
}

int main(int argc, char **argv) { 

    ros::init(argc,argv, "position_handler");
    ros::NodeHandle nh; 

    pos_client=nh.serviceClient<main_control_pkg::takeoff_serv>("/position");
    ros::Subscriber odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",10,odom_cbb);

    for(position &pos: positions){
        gotoposition(pos.x,pos.y,pos.z);
    }
}