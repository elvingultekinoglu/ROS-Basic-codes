#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <control_pkg/konum.h>
#include <control_pkg/motion_type.h>
#include <string>

geometry_msgs::PoseStamped pose_c;
control_pkg::konum srv; 
control_pkg::motion_type type; 

ros::ServiceClient type_client; 
std::string result(""); 
std::string circle("circle"); 

bool type_func(control_pkg::motion_type::Request &req, control_pkg::motion_type::Response &res){
    result.assign(req.option);
    ROS_INFO("Chosen option:%s", result.c_str());
    return true; 
}

int main(int argc, char **argv){ 

    ros::init(argc, argv, "circle");
    ros::NodeHandle nh;

    ros::ServiceClient client= nh.serviceClient<control_pkg::konum>("/konum");
    ros::ServiceServer type_ser=nh.advertiseService("/motion_type", type_func); 
    type_client= nh.serviceClient<control_pkg::motion_type>("/motion_type"); 

    ros::Rate looprate(20);

    while(ros::ok()){ 
        if (result.compare("circle")==0){ 
            ROS_INFO("Circle option is ready");
            for (float i=0; ros::ok() && i<=6.28; i=i+0.01){
                pose_c.pose.position.x=5*cos(i);
                pose_c.pose.position.y=5*sin(i);
                pose_c.pose.position.z=2; 
                srv.request.poseStamped = pose_c;
                client.call(srv);
                /*ROS_INFO("x:%f", pose_c.pose.position.x); 
                ROS_INFO("y:%f", pose_c.pose.position.y);
                ROS_INFO("x:%f", pose_c.pose.position.z);*/
                ros::Duration(0.010).sleep();
                
            }
        }else{
            ROS_INFO("w");
        }

        looprate.sleep();
        ros::spinOnce();
    }
    

    
}