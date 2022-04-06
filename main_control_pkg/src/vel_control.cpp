#include <ros/ros.h> 
#include <mavros_msgs/State.h> 
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h> 
#include <nav_msgs/Odometry.h> 
#include <main_control_pkg/takeoff_srv.h>
#include <main_control_pkg/takeoff_serv.h>
#include <mavros_msgs/ParamSet.h>
#include <geometry_msgs/TwistStamped.h>


geometry_msgs::TwistStamped vel_com;
mavros_msgs::State current_state;
mavros_msgs::SetMode mode;
mavros_msgs::CommandBool arm;
nav_msgs::Odometry odom; 
mavros_msgs::ParamSet param;
double takeoff_height;

bool isTakeoff; 

ros::ServiceClient param_client; 

void state_cb(const mavros_msgs::State::ConstPtr & msg ){ 
    current_state= *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr & msg){
    odom= *msg;
}

bool vel_cb(main_control_pkg::takeoff_srv::Request &req , main_control_pkg::takeoff_srv::Response &res){
    vel_com=req.twistStamped;
    return true; 
}

ros::ServiceClient arm_client;
ros::ServiceClient mode_client;

bool takeoff_func(main_control_pkg::takeoff_serv::Request &req, main_control_pkg::takeoff_serv::Response &res){
    
    if (req.isTakeoff){

        arm.request.value=true;
        arm_client.call(arm); 

        param.request.param_id="MIS_TAKEOFF_ALT";
        param.request.value.real=req.poseStamped.pose.position.z; 
        param_client.call(param); 

        mode.request.custom_mode="AUTO.TAKEOFF";
        mode_client.call(mode);
        takeoff_height=req.poseStamped.pose.position.z;
        isTakeoff=true; 

    }else { 

        mode.request.custom_mode="AUTO.LAND";
        mode_client.call(mode);
        isTakeoff=false;
    }
    
    return true; 
}

int main(int argc, char **argv){
    ros::init(argc,argv, "velocity_control");
    ros::NodeHandle nh; 

    ros::Subscriber state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::Publisher vel_pub=nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",10);
    mode_client=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client=nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::Subscriber odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",10,odom_cb);
    ros::ServiceServer takeoff_server=nh.advertiseService("/takeoff",takeoff_func);
    ros::ServiceServer velocity_srv=nh.advertiseService("/velocity", vel_cb);
    param_client=nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set"); 
    ros::ServiceServer position_ser=nh.advertiseService("/position",vel_cb);

    ros::Rate looprate(20);                  

    while(ros::ok()&&!current_state.connected){ 
        looprate.sleep();
        ros::spinOnce();
    }

    while(ros::ok()){
        
        ROS_INFO("%d", current_state.connected);

        while((!isTakeoff)&&ros::ok()){
            looprate.sleep();
            ros::spinOnce();
        }

        while(ros::ok()&&(odom.pose.pose.position.z<(takeoff_height-0.5))){
            looprate.sleep();
            ros::spinOnce(); 
        }

        for(int i=100; ros::ok()&&i>0; --i){
            vel_pub.publish(vel_com);
            ros::spinOnce();
            looprate.sleep();
        }

        mode.request.custom_mode="OFFBOARD";

        if(mode_client.call(mode)==true){
            ROS_INFO("Called setmode");
            if(mode.response.mode_sent==true){
                ROS_INFO("Success");
            }
        

            while (ros::ok()){
                ROS_INFO("x:%f, y:%f, z:%f, connected:%d, armed:%d, mode:%s", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, current_state.connected, current_state.armed, current_state.mode.c_str());

                if(!isTakeoff){
                    break;
                }

                vel_pub.publish(vel_com); 
                looprate.sleep();
                ros::spinOnce(); 
            }
        }
    }

}