#include <ros/ros.h> 
#include <mavros_msgs/State.h>  
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h> 
#include <nav_msgs/Odometry.h> 
#include <mavros_msgs/ExtendedState.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h> 
#include <vector> 

///message objects 

mavros_msgs::CommandBool arm; 
mavros_msgs::CommandTOL com; 
mavros_msgs::SetMode mode; 
mavros_msgs::State current_state; 
nav_msgs::Odometry odom; 
mavros_msgs::ParamSet param;
mavros_msgs::ExtendedState extended; 
geometry_msgs::PoseStamped pose; 
mavros_msgs::PositionTarget target; 

ros::Subscriber ex_state_sub; 
ros::ServiceClient arm_client;
ros::ServiceClient mode_client; 
ros::ServiceClient param_client; 
ros::Subscriber odom_sub;
ros::Publisher pos_pub;
ros::Subscriber state_sub; 
ros::ServiceClient takeoff_client; 
ros::Publisher target_publisher; 

//// variables 
float takeoff_altitude; 

void extended_cb(const mavros_msgs::ExtendedState::ConstPtr & msg){
    extended= *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr & msg ){ 
    current_state= *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr & msg){
    odom= *msg;
}

//// function for auto takeoff 
void takeoff_function(float altitude, float min_pitch, float yaw ){

//// disable certain parameters since this is simulator environment 
    param.request.param_id="COM_RC_IN_MODE"; 
    param.request.value.real=1; 
    param.request.param_id="NAV_RCL_ACT";
    param.request.value.real=0; 
    param.request.param_id="NAV_DLL_ACT";
    param.request.value.real=0; 
    param_client.call(param); 

    if ((extended.landed_state==1) || odom.pose.pose.position.z<10){

    ///// parameters to change for runway takeoff 
        com.request.altitude=altitude;
        com.request.min_pitch=min_pitch; 
        com.request.latitude=0;
        com.request.longitude=0;
        com.request.yaw=yaw;

    ////// to check whether takeoff is okey or not 
        if(takeoff_client.call(com)){
            ROS_INFO("Takeoff is successfull!");
        }else{
            ROS_WARN("Takeoff is fail!"); 
        }

        takeoff_altitude=altitude; 
        
    }
}

//// function fot arming 
void arming_function(){
    
    if (current_state.armed==false){
        arm.request.value=true;
        arm_client.call(arm);

        ROS_INFO("Arming is done."); 
    }
}


//// setting type_mask for gliding setpoint 
void setting_setpoints(){
    target.type_mask=292; /// to set gliding setpoints 
    target.coordinate_frame=1; 

    target_publisher.publish(target); 
}


//// setting offboard as custom_mode 
void set_offboard_mode(){
    mode.request.base_mode=0; 
    mode.request.custom_mode="OFFBOARD"; 
    if(mode_client.call(mode)){
        ROS_INFO("Custom mode is successfull!");
    }else {
        ROS_WARN("Custom mode is error!");
    }
}

//// setting offboard mode properties
void setting_offboard_mode(){
    odom.pose.pose.position.x=0;
    odom.pose.pose.position.y=0;
    odom.pose.pose.position.z=takeoff_altitude; 

    pos_pub.publish(pose); 
}

//// pose_stamped function
void pose_stamped(double x, double y, double z){
    pose.pose.position.x=x;
    pose.pose.position.y=y;
    pose.pose.position.z=z; 

    pos_pub.publish(pose);
}


//// assigning setpoints
void gotoposition(float x, float y, float z){
    
    struct desired_values {
        float x;
        float y;
        float z; 
    };  

    std::vector<desired_values> positions; 

    ros::Rate looprate(20); 

    positions.reserve(20);
    positions.push_back({x,y,z});
    for(desired_values &des : positions){
        gotoposition(des.x, des.y, des.z); ///bunu check et !!! 
        ros::Duration(10.0).sleep();
        ros::spinOnce(); 
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"takeoff_trial");
    ros::NodeHandle nh; 

//// subscribers, publishers, services  

    ex_state_sub=nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",10,extended_cb);
    state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    mode_client=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client=nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",10,odom_cb);
    param_client=nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    pos_pub=nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    takeoff_client=nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    target_publisher=nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10); 

    ros::Rate looprate(20); 

    while(ros::ok()&&!current_state.connected){ 
        looprate.sleep();
        ros::spinOnce();
    }

    
    setting_setpoints(); 
    arming_function();
    takeoff_function(30.0, 30.0, 50.0); 
    
    

    while(ros::ok()){
        
        mode.request.base_mode=0; 
        mode.request.custom_mode="OFFBOARD"; 
        if(mode_client.call(mode)){
            ROS_INFO("Custom mode is successfull!");
        }else {
            ROS_WARN("Custom mode is error!");
        }

        ROS_INFO("x:%f, y:%f, z:%f, connected:%d, armed:%d, mode:%s", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, current_state.connected, current_state.armed, current_state.mode.c_str()); 
        looprate.sleep(); 
        ros::spinOnce(); 
    }

}