#include <ros/ros.h> 
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h> 
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h> 
#include <nav_msgs/Odometry.h> 
#include <main_control_pkg/takeoff_serv.h>
#include <mavros_msgs/ParamSet.h>

geometry_msgs::PoseStamped pose_com;
mavros_msgs::State current_state;
mavros_msgs::SetMode mode;
mavros_msgs::CommandBool arm;
nav_msgs::Odometry odom; 
mavros_msgs::ParamSet param;

bool isTakeoff; 

ros::ServiceClient param_client; 

void state_cb(const mavros_msgs::State::ConstPtr & msg ){ 
    current_state= *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr & msg){
    odom= *msg;
}

bool pos_cb(main_control_pkg::takeoff_serv::Request &req , main_control_pkg::takeoff_serv::Response &res){
    pose_com=req.poseStamped;
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
        pose_com=req.poseStamped;
        isTakeoff=true; 

    }else { 

        mode.request.custom_mode="AUTO.LAND";
        mode_client.call(mode);
        isTakeoff=false;
    }
    
    return true; 
}

int main(int argc, char **argv){
    ros::init(argc,argv, "position_control");
    ros::NodeHandle nh; 

    ros::Subscriber state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::Publisher pose_pub=nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    mode_client=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client=nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::Subscriber odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",10,odom_cb);
    ros::ServiceServer takeoff_server=nh.advertiseService("/takeoff",takeoff_func);
    param_client=nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set"); 
    ros::ServiceServer position_ser=nh.advertiseService("/position",pos_cb);

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

        while(ros::ok()&&(odom.pose.pose.position.z<(pose_com.pose.position.z-0.5))){
            looprate.sleep();
            ros::spinOnce(); 
        }

        for(int i=100; ros::ok()&&i>0; --i){
            pose_pub.publish(pose_com);
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

                pose_pub.publish(pose_com); 
                looprate.sleep();
                ros::spinOnce(); 
            }
        }
    }

}