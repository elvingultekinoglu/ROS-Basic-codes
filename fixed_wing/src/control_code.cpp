#include <ros/ros.h>
#include <mavros_msgs/State.h> 
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h> 
#include <nav_msgs/Odometry.h> 
#include <mavros_msgs/ParamSet.h>
#include <geometry_msgs/PoseStamped.h> 
#include <fixed_wing/takeoff_srv.h>
#include <fixed_wing/position_srv.h>
#include <mavros_msgs/PositionTarget.h> 


mavros_msgs::PositionTarget target; 
geometry_msgs::PoseStamped pose;
mavros_msgs::State current_state;
mavros_msgs::SetMode mode;
mavros_msgs::CommandBool arm;
nav_msgs::Odometry odom; 
mavros_msgs::ParamSet param; 

void state_cb(const mavros_msgs::State::ConstPtr & msg ){ 
    current_state= *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr & msg){
    odom= *msg;
}

bool istakeoff; 
ros::ServiceClient arm_client;
ros::ServiceClient mode_client; 
ros::ServiceClient param_client; 

bool takeoff_func(fixed_wing::takeoff_srv::Request &req, fixed_wing::takeoff_srv::Response &res ){
    if(req.takeoff){

        arm.request.value=true;
        arm_client.call(arm);

        param.request.param_id="RWTO_TKOFF"; 
        param.request.value.real=1; 
        param.request.param_id="COM_RC_IN_MODE"; 
        param.request.value.real=1; 
        param.request.param_id="NAV_RCL_ACT";
        param.request.value.real=0; 
        param.request.param_id="NAV_DLL_ACT";
        param.request.value.real=0; 

        param_client.call(param);

        mode.request.custom_mode="AUTO.TAKEOFF";
        mode_client.call(mode); 

        istakeoff=true;        


    } else {

        mode.request.custom_mode="AUTO.LAND";
        mode_client.call(mode);
        istakeoff=false;
    }

    return true; 
}


/*void gotoposition(double altitude){
    pose.pose.position.z=altitude; 
    
}*/

bool position_func(fixed_wing::position_srv::Request &req, fixed_wing::position_srv::Response &res){
    
    if (istakeoff==true){
        target.position=req.poseStamped.pose.position; 
    }
    
    return true; 
}

int main(int argc, char **argv){
    ros::init(argc,argv, "takeoff_code"); 
    ros::NodeHandle nh; 

    ros::Subscriber state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    /*ros::Publisher vel_pub=nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",10);*/
    mode_client=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client=nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::Subscriber odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",10,odom_cb);
    param_client=nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set"); 
    ros::ServiceServer takeoff_server=nh.advertiseService("/takeoff",takeoff_func);
    ros::Publisher pos_pub=nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::ServiceServer position_server=nh.advertiseService("/position",position_func); 
    ros::Publisher target_publisher=nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

    ros::Rate looprate(20);                  

    while(ros::ok()&&!current_state.connected){ 
        looprate.sleep();
        ros::spinOnce();
    }


    while (ros::ok()){    

        ROS_INFO("%d, %s", current_state.connected, current_state.mode.c_str());

        while((!istakeoff)&&ros::ok()){
            looprate.sleep();
            ros::spinOnce();
        }

        while(ros::ok()&&(odom.pose.pose.position.z<(15-0.5))){
            looprate.sleep();
            ros::spinOnce(); 
        }

        for(int i=100; ros::ok()&&i>0; --i){
            pos_pub.publish(pose);
            ros::spinOnce();
            looprate.sleep();
        }

        mode.request.custom_mode="OFFBOARD";
        target.type_mask=292; /// to set gliding setpoints 
        target.coordinate_frame=1; 


        if(mode_client.call(mode)==true){
            ROS_INFO("Called setmode");
            if(mode.response.mode_sent==true){
                ROS_INFO("Success");
            }               
        
            while(ros::ok()){
                ROS_INFO("x:%f, y:%f, z:%f, connected:%d, armed:%d, mode:%s", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, current_state.connected, current_state.armed, current_state.mode.c_str());
        
                if(!istakeoff){
                    break;
                }

                pos_pub.publish(pose);
                target_publisher.publish(target);
                looprate.sleep();
                ros::spinOnce(); 
            } 

        } 
    }

    

}