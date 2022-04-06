#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h> 
#include <main_control_pkg/takeoff_srv.h> 
#include <mavros_msgs/State.h> 


geometry_msgs::TwistStamped velo;
nav_msgs::Odometry odom; 
main_control_pkg::takeoff_srv velo_srv; 
mavros_msgs::State current_state; 

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg){
    odom= *msg;
}

void state_cb(const mavros_msgs::State::ConstPtr & msg ){ 
    current_state= *msg;
}


ros::Subscriber odom_sub;
ros::ServiceClient client;
ros::Subscriber state_sub; 

void ayrit(double desired_x, double desired_y, double desired_z) {
    
    double x_location, y_location, z_location; 
    double x_difference, y_difference, z_difference; 
    double kp_x=1.2;
    double kp_y=1.2;
    double kp_z=0.8;
    double current_time;
    double last_time; 
    double dt; 
    double kd_x=0.02;
    double kd_y=0.02; 
    double kd_z=0.01;
    double ki_x=0.00001;
    double ki_y=0.00001;
    double ki_z=0.00001; 
    double p_x,p_y,p_z,i_x,i_y,i_z,d_x,d_y,d_z;
    double last_error_x,last_error_y,last_error_z;
    double integral_x,integral_y,integral_z; 

    ros::Rate looprate(50);

    last_time=ros::Time::now().toSec();


    while (ros::ok()){ 

        current_time=ros::Time::now().toSec();

        if(current_time-last_time>10){
            break; 
        }

        dt=ros::Time::now().toSec()-last_time;

        x_location=odom.pose.pose.position.x; 
        y_location=odom.pose.pose.position.y; 
        z_location=odom.pose.pose.position.z; 

        x_difference=desired_x - x_location; 
        y_difference=desired_y - y_location; 
        z_difference=desired_z - z_location;

        integral_x+=x_difference;
        integral_y+=y_difference;
        integral_z+=z_difference;

        if ((last_error_x<0 && integral_x>0) || (last_error_x>0 && integral_x<0)){
            integral_x=0;
        }

        if ((last_error_y<0 && integral_y>0) || (last_error_y>0 && integral_y<0)){
            integral_y=0;
        }

        if ((last_error_z<0 && integral_z>0) || (last_error_z>0 && integral_z<0)){
            integral_z=0;
        }

        p_x=kp_x*x_difference;
        p_y=kp_y*y_difference;
        p_z=kp_z*z_difference;

        d_x=kd_x*(x_difference-last_error_x)/dt; 
        d_y=kd_y*(y_difference-last_error_y)/dt; 
        d_z=kd_z*(z_difference-last_error_z)/dt;

        i_x=ki_x*integral_x; 
        i_y=ki_y*integral_y;
        i_z=ki_z*integral_z;

        velo.twist.linear.x=p_x+i_x;
        velo.twist.linear.y=p_y+i_y;
        velo.twist.linear.z=p_z+i_z;
        

        velo_srv.request.twistStamped = velo; 
        client.call(velo_srv);

        last_error_x=x_difference;
        last_error_y=y_difference;
        last_error_y=z_difference;

        ROS_INFO("x location=%f, y position=%f, z position=%f , connected=%d, current time=%f, last time=%f, difference=%f",
         x_location, y_location, z_location, current_state.connected, current_time, last_time, current_time-last_time);

        ros::spinOnce();
        looprate.sleep();


    } 

    ros::Duration(0.5).sleep(); 
    
}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "org_code");
    ros::NodeHandle nh; 

    odom_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local",10,odom_cb);
    client= nh.serviceClient<main_control_pkg::takeoff_srv>("/velocity");
    state_sub=nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);


    ayrit(5,0,2);
    ayrit(5,5,2);
    ayrit(0,5,2);
    ayrit(0,0,2);
    ayrit(0,0,7);
    ayrit(5,0,7);
    ayrit(5,5,7);
    ayrit(0,5,7);
    ayrit(0,0,7); 
    ayrit(0,5,7);
    ayrit(0,5,2);
    ayrit(5,5,2);
    ayrit(5,5,7);
    ayrit(5,0,7);
    ayrit(5,0,2);
    

}