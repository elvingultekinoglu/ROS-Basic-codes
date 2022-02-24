#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <control_pkg/konum.h>
#include <control_pkg/velo.h>
#include <geometry_msgs/TwistStamped.h>

double desired_location=5; 
double x_location;
double y_location;
double z_location;
double kp=1.5; 
double ki=0.2; //en başta ki ve kd yi 0 verip dene sorun görmezsen devam et  
double kd=0.2; 
double difference; 
double last_error; 
double integral; 
double p,i,d; 
double current_time;
double last_time; 
double dt; 

nav_msgs::Odometry konum;
geometry_msgs::TwistStamped hiz; 
control_pkg::velo velo; 


void call_b(const nav_msgs::Odometry::ConstPtr & msg){
    konum= *msg; 
}

int main(int argc, char** argv){

    ros::init(argc, argv, "location_pid");
    ros::NodeHandle nh;

    ros::Subscriber konum_sub=nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, call_b);
    ros::ServiceClient client= nh.serviceClient<control_pkg::velo>("/velocity");

    ros::Rate looprate(20); 

    

    while (ros::ok()) { 
 
        /*z_location= konum.pose.pose.position.z;
        difference=desired_location-z_location;
        hiz.twist.linear.z=kp*difference; 
        velo.request.twistStamped=hiz; 
        client.call(velo);
        ROS_INFO("Z Location:%f", z_location);*/ //sadece p kısmı ile drone u belli yükseklikte tutmak 
        
        current_time= ros::Time::now().toSec(); 
        dt= current_time - last_time; 

        z_location= konum.pose.pose.position.z;
        last_error= difference; 
        difference= desired_location - z_location;
        integral += difference; 
        
        if (last_error<0 && difference>0){ 
            integral=0;
        } //bunun amacı anti-windup uygulamak 

        if (last_error>0 && difference<0){ 
            integral=0;
        } 

        p= kp*difference; 
        d= kd*(difference-last_error)/dt; 
        i= ki*integral; 

        hiz.twist.linear.z= p+d+i;
        velo.request.twistStamped= hiz; 
        client.call(velo);

        last_time = current_time; 


        ROS_INFO("Z Location:%f", z_location); 
        ros::spinOnce();
        looprate.sleep(); 


    }
    

    
}