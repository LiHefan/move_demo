#include "std_msgs/Float64MultiArray.h" 
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_position_pub");
  ros::NodeHandle n;
  ros::Publisher object_position_pub = n.advertise<std_msgs::Float64MultiArray>("object_position", 1000); 
  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    std_msgs::Float64MultiArray msg;
    std::vector<double> object_position = {0.75, 0.38};
    msg.data = object_position;
 
    ROS_INFO("I have published array data"); 
    object_position_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
