#include <ros/ros.h>
#include <nav_msgs/Odometry.h> 
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h> 
#include <mavros_msgs/CommandBool.h> 
#include <geometry_msgs/PoseStamped.h>
#include <control_pkg/konum.h> 
#include <control_pkg/arm_ser.h> 
#include <control_pkg/motion_type.h> 
#include <control_pkg/takeoff_serv.h>
#include <geometry_msgs/Quaternion.h> //kullanılmayan
#include <tf2/LinearMath/Quaternion.h> //kullanılmayan
#include <geometry_msgs/TwistStamped.h> //kullanılmayan 
#include <control_pkg/velo.h> 
#include <mavros_msgs/ParamSet.h>


geometry_msgs::PoseStamped pose_com; 
mavros_msgs::State current_state; 
nav_msgs::Odometry odom;
mavros_msgs::CommandBool arm;
mavros_msgs::SetMode mode;  
/*static tf2::Quaternion quat;*/
geometry_msgs::TwistStamped vel; 
mavros_msgs::ParamSet param_cmd; 

bool isTakeoff; 


void state_cb(const mavros_msgs::State::ConstPtr & msg){ 
    current_state = *msg; 
}

void call_b(const nav_msgs::Odometry::ConstPtr & msg){
    odom = *msg; 
}

bool location_cb(control_pkg::konum::Request &req, control_pkg::konum::Response &res){
    pose_com = req.poseStamped;
    return true; 
}

bool velocity_cb(control_pkg::velo::Request &req, control_pkg::velo::Response &res){ 
    vel=req.twistStamped;
    return true; 
}


ros::ServiceClient arm_client;

bool arming_func(control_pkg::arm_ser::Request &req, control_pkg::arm_ser::Response &res){ 
    arm.request.value = req.arm_value; 
    if (arm_client.call(arm)==true){
        if(req.arm_value==true){
            ROS_INFO("Arming is success"); 
        } else { 
            ROS_INFO("Arming is not success"); 
        }   
    }
    
    return true; 
}


ros::ServiceClient mode_client; 
ros::ServiceClient param_client;

bool takeoff_func(control_pkg::takeoff_serv::Request &req, control_pkg::takeoff_serv::Response &res) {
    
    if(req.isTakeoff){ 

        arm.request.value=true; 
        arm_client.call(arm);

        param_cmd.request.param_id="MIS_TAKEOFF_ALT";
        param_cmd.request.value.real=req.pose.pose.position.z; 
        param_client.call(param_cmd);

        mode.request.custom_mode = "AUTO.TAKEOFF";
        mode_client.call(mode); 
        pose_com=req.pose;
        isTakeoff=true; 
    } else { 
        mode.request.custom_mode="AUTO.LAND";
        mode_client.call(mode);
        isTakeoff=false; 
    }
    return true; 
}

int main(int argc, char **argv){ 
    ros::init(argc, argv, "takeoff_example");
    ros::NodeHandle nh;
    
    ros::Subscriber odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, call_b);
    ros::Subscriber state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    arm_client=nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mode_client=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    ros::Publisher pose_pub=nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::ServiceServer location=nh.advertiseService("/konum",location_cb); 
    ros::ServiceServer armm=nh.advertiseService("/arm_ser", arming_func);  
    ros::Publisher velocity_pub=nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",10);
    ros::ServiceServer velocity_ser=nh.advertiseService("/velocity",velocity_cb); 
    param_client=nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set"); 
    ros::ServiceServer takeoff_server=nh.advertiseService("/takeoff", takeoff_func);
    ros::Rate rate(20);

    while (ros::ok() && !current_state.connected){ 
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){

        ROS_INFO("%d", current_state.connected);

        while ((!isTakeoff)&&ros::ok()){
            rate.sleep();
            ros::spinOnce();
        }
    
        while( ros::ok() && (odom.pose.pose.position.z<(pose_com.pose.position.z-0.5))){
            rate.sleep();
            ros::spinOnce();
        }



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

            /*arm.request.value=true;

            if (arm_client.call(arm)==true){ 
                if(arm.response.result==0){
                    ROS_INFO("Arming is successfull");
                }
            }*/
    

            while(ros::ok()){

                ROS_INFO("x:%f, y:%f, z:%f, connected:%d, armed:%d, mode:%s", odom.pose.pose.position.x , odom.pose.pose.position.y, odom.pose.pose.position.z, current_state.connected, current_state.armed, current_state.mode.c_str()); 

                if (!isTakeoff){
                    break;
                }

                pose_pub.publish(pose_com);
                /*velocity_pub.publish(vel);*/
                rate.sleep();
                ros::spinOnce();

            }
        
        }
    }


}