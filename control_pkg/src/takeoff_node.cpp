#include <ros/ros.h>
#include <nav_msgs/Odometry.h> 
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h> 
#include <mavros_msgs/CommandBool.h> 
#include <geometry_msgs/PoseStamped.h>

mavros_msgs::State current_state; 
nav_msgs::Odometry odom;
mavros_msgs::CommandBool arm;
mavros_msgs::SetMode mode;
geometry_msgs::PoseStamped pose_com;

void state_cb(const mavros_msgs::State::ConstPtr & msg){ 
    current_state = *msg; 
}

void call_b(const nav_msgs::Odometry::ConstPtr & msg){
    odom = *msg; 
}

int main(int argc, char **argv){ 
    ros::init(argc, argv, "takeoff_example");
    ros::NodeHandle nh;
    
    ros::Subscriber odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, call_b);
    ros::Subscriber state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::ServiceClient arm_client=nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient mode_client=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Publisher pose_pub=nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);

    ros::Rate rate(20);

    while (ros::ok() && !current_state.connected){ 
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("%d", current_state.connected);

    pose_com.pose.position.x=0;
    pose_com.pose.position.y=0;
    pose_com.pose.position.z=0;

    for(int i=100; ros::ok() && i>0; --i){
        pose_pub.publish(pose_com);
        ros::spinOnce();
        rate.sleep();
    }

    mode.request.custom_mode="OFFBOARD";

    if(mode_client.call(mode)==true){
        ROS_INFO("called setmode");
        if(mode.response.mode_sent==true){
            ROS_INFO("Success");
        }

        arm.request.value=true;

        if (arm_client.call(arm)==true){ 
            if(arm.response.result==0){
                ROS_INFO("Arming is successfull");
            }
        }

        pose_com.pose.position.x=0;
        pose_com.pose.position.y=0;
        pose_com.pose.position.z=2;

        while(ros::ok()){
            ROS_INFO("%f", "%f", "%f", "%d", "%d", "%s", odom.pose.pose.position.x , odom.pose.pose.position.y, odom.pose.pose.position.z, 
            current_state.connected, current_state.armed, current_state.mode);

            pose_pub.publish(pose_com);
            rate.sleep();
            ros::spinOnce();            
        }
    }


}