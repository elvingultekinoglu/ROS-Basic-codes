#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <control_pkg/konum.h>

control_pkg::konum serv;
geometry_msgs::PoseStamped orient_c; 
tf2::Quaternion quat;

int main(int argc, char ** argv){

    ros::init(argc, argv, "orientation");
    ros::NodeHandle nh; 
    ros::ServiceClient client=nh.serviceClient<control_pkg::konum>("/konum"); 
    ros::Rate looprate(20); 

    while (ros::ok()){

        orient_c.pose.position.z=3; 

        double yaw=240;
        double roll=0;
        double pitch=0;
        
        quat.setRPY(roll,pitch,yaw*(3.14/180)); 
        quat=quat.normalize();
        orient_c.pose.orientation.x=quat.x();
        orient_c.pose.orientation.y=quat.y();
        orient_c.pose.orientation.z=quat.z();
        orient_c.pose.orientation.w=quat.w();

        serv.request.poseStamped = orient_c;
        client.call(serv); 

        /*ROS_INFO("Orientation is successfull");*/

        ros::spinOnce();
    }
}
