#include <ros/ros.h> 
#include <mavros_msgs/State.h>  
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

mavros_msgs::CommandBool arm; 
mavros_msgs::State current_state; 
mavros_msgs::SetMode mode;

void state_cb(const mavros_msgs::State::ConstPtr & msg ){ 
    current_state= *msg;
}

int main(int argc, char **argv){

    ros::init(argc,argv,"arming_deneme_kodu");
    ros::NodeHandle nh; 

    ros::Subscriber state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::ServiceClient arm_client=nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient mode_client=nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Rate looprate(20); 

    while(ros::ok()&&!current_state.connected){ 
        looprate.sleep();
        ros::spinOnce();
    }

    while(ros::ok()){
        ROS_INFO("armed:%d, mode:%s", current_state.armed, current_state.mode.c_str()); 
        if(current_state.armed){
        
            ROS_INFO("Arming successful!"); 
        
        } else {
            ROS_INFO("Arming is not successful!"); 
        }
        looprate.sleep();
        ros::spinOnce(); 
    }

} 
