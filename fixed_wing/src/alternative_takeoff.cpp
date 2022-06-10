#include <ros/ros.h>
#include <mavros_msgs/State.h>  
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h> 
#include <nav_msgs/Odometry.h> 
#include <fixed_wing/takeoff_srv.h> 

mavros_msgs::CommandBool arm; 
mavros_msgs::CommandTOL com; 
mavros_msgs::SetMode mode; 
mavros_msgs::State current_state; 
nav_msgs::Odometry odom; 
mavros_msgs::ParamSet param; 

ros::ServiceClient arm_client;
ros::ServiceClient param_client; 
ros::ServiceClient mode_client; 

void state_cb(const mavros_msgs::State::ConstPtr & msg ){ 
    current_state= *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr & msg){
    odom= *msg;
} 

///////////////////////////////////
//////////NOT-BU KOD AUTO.TAKEOFF İLE AYNI MANTIKTA ÇALIŞIP
//////////EN SONUNDA AUTO.TAKEOFF VERİR ANCAK BİR SÜRE 
//////////SONRA TAKLA ATTIRIYOR. O YÜZDEN KULLANMAYIN!!!!!

int main(int argc, char **argv){
    ros::init(argc,argv, "alternative_takeoff"); 
    ros::NodeHandle nh; 

    ros::Rate looprate(20); 

    arm_client=nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mode_client=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"); 
    param_client=nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set"); 
    ros::ServiceClient takeoff_client=nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff"); 
    ros::Subscriber state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb); 
    ros::Subscriber odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",10,odom_cb);

    param.request.param_id="COM_RC_IN_MODE"; 
    param.request.value.real=1; 
    param.request.param_id="NAV_RCL_ACT";
    param.request.value.real=0; 
    param.request.param_id="NAV_DLL_ACT";
    param.request.value.real=0; 

    param_client.call(param);

    while(ros::ok()&&!current_state.connected){ 
        looprate.sleep();
        ros::spinOnce();
    }
    
    //////////////////////////////////
    /////////////OFFBOARD////////////

    mode.request.base_mode=0; 
    mode.request.custom_mode="OFFBOARD"; 
    if(mode_client.call(mode)){
        ROS_INFO("Custom mode is successfull!");
    }else {
        ROS_WARN("Custom mode is error!");
    }

    ////////////////////////////////
    /////////////ARM///////////////

    arm.request.value=true; 
    if(arm_client.call(arm)){
        ROS_INFO("Arming is successfull!");
    } else{
        ROS_WARN("Armins is error!");
    } 

    //////////////////////////////
    ///////////TAKEOFF///////////

    com.request.altitude=50;
    com.request.min_pitch=20;
    com.request.latitude=0;
    com.request.longitude=0;
    com.request.yaw=0; 
    if(takeoff_client.call(com)){
        ROS_INFO("Takeoff is successfull!");
    }else{
        ROS_WARN("Takeoff is fail!"); 
    }

    while(ros::ok()){

        ROS_INFO("x:%f, y:%f, z:%f, connected:%d, armed:%d, mode:%s", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, current_state.connected, current_state.armed, current_state.mode.c_str()); 
        looprate.sleep(); 
        ros::spinOnce(); 
    }


}