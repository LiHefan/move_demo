#include "std_msgs/Float64.h" 
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tns_pub");
  ros::NodeHandle n;
  ros::Publisher tns_pub = n.advertise<std_msgs::Float64>("tns", 1000); 
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::Float64 msg;
    double distance = 0.1;
    msg.data = distance;

    ROS_INFO("I have published double data [%f]",msg.data); 
    tns_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}