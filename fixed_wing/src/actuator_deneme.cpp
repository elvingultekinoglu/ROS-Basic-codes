#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
 
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

ros::Publisher act_pub; 
void actuator_function() {
    mavros_msgs::ActuatorControl setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.group_mix = 0;
    setpoint.controls[0] = 0; 
    setpoint.controls[1] = 0; 
    setpoint.controls[2] = 0; 
    setpoint.controls[3] = 1;  
    setpoint.controls[4] = 0; 
    setpoint.controls[5] = 0;
    setpoint.controls[6] = 0; 
    setpoint.controls[7] = 0;

    act_pub.publish(setpoint);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    act_pub = nh.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 20;
    pose.pose.position.z = 20;

    mavros_msgs::ActuatorControl setpoint;
    setpoint.header.stamp = ros::Time::now();
    setpoint.group_mix = 0;
    setpoint.controls[0] = 0; 
    setpoint.controls[1] = 0; 
    setpoint.controls[2] = 0; 
    setpoint.controls[3] = 1;  
    setpoint.controls[4] = 0; 
    setpoint.controls[5] = 0;
    setpoint.controls[6] = 0; 
    setpoint.controls[7] = 0;

    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(pose);
        act_pub.publish(setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ros::Time last_request = ros::Time::now();
    ros::Time last_request_arm = ros::Time::now();

    while(ros::ok()){
        // act_pub.publish(setpoint);
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(0.5))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        if( !current_state.armed &&
            (ros::Time::now() - last_request_arm > ros::Duration(1))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request_arm = ros::Time::now();
            }
        
        // local_pos_pub.publish(pose);
        // act_pub.publish(setpoint);
        // local_pos_pub.publish(pose);
        actuator_function(); 
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
