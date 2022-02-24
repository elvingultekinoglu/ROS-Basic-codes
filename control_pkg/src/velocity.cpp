#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h> 
#include <control_pkg/velo.h> 

geometry_msgs::TwistStamped veloc; 
control_pkg::velo srv; 

int main( int argc, char** argv){

    ros::init(argc, argv, "velocity");
    ros::NodeHandle nh; 

    ros::ServiceClient client= nh.serviceClient<control_pkg::velo>("/velocity");

    ros::Rate looprate(20); 

    while (ros::ok()){ 

        veloc.twist.linear.z=1; 
        srv.request.twistStamped=veloc; 
        client.call(srv); 
        ros::spinOnce();
    }

    
}
