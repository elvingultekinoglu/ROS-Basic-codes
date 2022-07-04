#include <ros/ros.h>
#include <std_msgs/Bool.h> 
#include <std_msgs/Float64.h>

std_msgs::Bool bool_msg1;
std_msgs::Bool bool_msg2; 
std_msgs::Bool bool_msg3; 
std_msgs::Float64 float1;

int main(int argc, char **argv){

    ros::init(argc, argv, "publisher_code");
    ros::NodeHandle nh;

    ros::Publisher bool_pub1=nh.advertise<std_msgs::Bool>("/bools_from_cv1",10);
    ros::Publisher bool_pub2=nh.advertise<std_msgs::Bool>("/bools_from_cv2",10);
    ros::Publisher bool_pub3=nh.advertise<std_msgs::Bool>("/bools_from_cv3",10);
    ros::Publisher float_pub=nh.advertise<std_msgs::Float64>("/floats_from_cv",10); 

    ros::Rate rate(20);

    bool_msg1.data=true;
    bool_msg2.data=true; 
    bool_msg3.data=true; 

    float1.data= 100.0; 

    while(ros::ok()) {

        bool_pub1.publish(bool_msg1);
        bool_pub2.publish(bool_msg2);
        bool_pub3.publish(bool_msg3);
        float_pub.publish(float1); 

        rate.sleep();
        ros::spinOnce(); 
        
    }

}