#include <ros/ros.h>
#include <fixed_wing/pos_handler_srv.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::PoseStamped pose; 
nav_msgs::Odometry odom; 
fixed_wing::pos_handler_srv srv; 

void odom_cb(const nav_msgs::Odometry::ConstPtr & msg){
    odom= *msg;
}

ros::Subscriber odom_sub;
ros::Publisher pose_pub;
ros::ServiceClient pos_client; 

#define radius 2 

void gotoposition(double x, double y, double z){

    srv.request.poseStamped.pose.position.x=x;
    srv.request.poseStamped.pose.position.y=y;
    srv.request.poseStamped.pose.position.z=z; 

    ros::Rate looprate(20); 
    pos_client.call(srv); 
}


int main(int argc, char** argv){
    ros::init(argc, argv, "pos_handler_code"); 
    ros::NodeHandle nh;

    pos_client=nh.serviceClient<fixed_wing::pos_handler_srv>("/pos_handler"); 
    odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",10,odom_cb);
    pose_pub=nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);

    ros::Rate looprate(20); 
    
    while(ros::ok()){

        gotoposition(100.0,0.0,100.0);
        ros::spinOnce();
        looprate.sleep(); 
    }

}