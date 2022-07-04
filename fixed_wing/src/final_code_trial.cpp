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
#include <fixed_wing/takeoff_land_srv.h> 
#include <fixed_wing/position_srv.h>
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
ros::ServiceServer takeoff_server; 
ros::ServiceClient land_client; 
ros::ServiceServer land_server; 
ros::ServiceServer position_server; 

//// subscriber functions
void extended_cb(const mavros_msgs::ExtendedState::ConstPtr & msg){
    extended= *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr & msg ){ 
    current_state= *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr & msg){
    odom= *msg;
}

bool istakeoff; 


//// function for takeoff service
bool takeoff_function(fixed_wing::takeoff_land_srv::Request &req, fixed_wing::takeoff_land_srv::Response &res){
    
    
    //// parameters to disable for simulation environment 
    param.request.param_id="COM_RC_IN_MODE"; 
    param.request.value.real=1; 
    param.request.param_id="NAV_RCL_ACT";
    param.request.value.real=0; 
    param.request.param_id="NAV_DLL_ACT";
    param.request.value.real=0; 
    param_client.call(param); 

    if(req.is_takeoff==true){
        
        if((extended.landed_state==1) || odom.pose.pose.position.z<10){
            
            arm.request.value=true;
            arm_client.call(arm);

            mode.request.base_mode=0; 
            mode.request.custom_mode="OFFBOARD";
            mode_client.call(mode); 
            
            com.request.altitude=50.0;
            com.request.min_pitch=30.0; 
            com.request.latitude=0.0;
            com.request.longitude=0.0;
            com.request.yaw=0.0;

            if(takeoff_client.call(com)){
                ROS_INFO("Takeoff is successfull!");
            }else{
                ROS_WARN("Takeoff is fail!"); 
            }

            istakeoff=true; 
        } else {
            istakeoff=false; 
        }
    } 
    return true; 
}


//// function for land service 
bool land_function(fixed_wing::takeoff_land_srv::Request &req, fixed_wing::takeoff_land_srv::Response &res){

    
    //// parameters to disable for simulation environment 
    param.request.param_id="COM_RC_IN_MODE"; 
    param.request.value.real=1; 
    param.request.param_id="NAV_RCL_ACT";
    param.request.value.real=0; 
    param.request.param_id="NAV_DLL_ACT";
    param.request.value.real=0; 
    param_client.call(param); 

    if (req.is_land==true){
        
        com.request.altitude=50.0;
        com.request.min_pitch=0.0; 
        com.request.latitude=0.0;
        com.request.longitude=0.0;
        com.request.yaw=90.0;

        if(land_client.call(com)){
            ROS_INFO("Landing is successfull!");
        } else {
            ROS_INFO("Landing is fail!");
        } 

        istakeoff=false;
    } else {
        istakeoff=true; 
    }

    return true; 
}

//// pose_stamped function
void pose_stamped(double x, double y, double z){
    pose.pose.position.x=x;
    pose.pose.position.y=y;
    pose.pose.position.z=z; 

    pos_pub.publish(pose);
} 

//// function to assign offboard setpoints 
bool position_func(fixed_wing::position_srv::Request &req, fixed_wing::position_srv::Response &res){
    
    struct desired_values {
        float x;
        float y;
        float z; 
    };  
    
    
    std::vector<desired_values> positions; 
    
    ros::Rate looprate(20); 

    positions.reserve(20);
    positions.push_back({req.poseStamped.pose.position.x,req.poseStamped.pose.position.y,
    req.poseStamped.pose.position.z});

    for(desired_values &des : positions){
        pose_stamped(des.x, des.y, des.z); 
        ros::Duration(10.0).sleep();
        ros::spinOnce(); 
    }

    return true; 
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
    land_client=nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land"); 
    target_publisher=nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10); 
    takeoff_server=nh.advertiseService("/takeoff", takeoff_function); 
    land_server=nh.advertiseService("/land", land_function); 
    position_server=nh.advertiseService("/position",position_func);

    ros::Rate looprate(20); 

    while(ros::ok()&&!current_state.connected){ 
        looprate.sleep();
        ros::spinOnce();
    }

    target.type_mask=292; /// to set gliding setpoints 
    target.coordinate_frame=1; 
    

    while (ros::ok()){    

        ROS_INFO("%d, %s", current_state.connected, current_state.mode.c_str());

        while((!istakeoff)&&ros::ok()){
            looprate.sleep();
            ros::spinOnce();
        }

        while(ros::ok()&&(odom.pose.pose.position.z<(5-0.5))){
            looprate.sleep();
            ros::spinOnce(); 
        }

        for(int i=100; ros::ok()&&i>0; --i){
            pos_pub.publish(pose);
            ros::spinOnce();
            looprate.sleep();
        }


        mode.request.custom_mode="OFFBOARD";
        pose.pose.position.z=50; //// bunu takeoff altitude'u ile aynı değer yapın 


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
