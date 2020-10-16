#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    ROS_INFO("I heard [%f], [%f],[%f]", msg->data[0], msg->data[1], msg->data[2]);
    double i = msg->data[0];
    ROS_INFO("Copy [%f]",i); 
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("gripper_position", 1000, chatterCallback);


    ros::spin();
    ROS_INFO("Heard.");



    return 0;
}