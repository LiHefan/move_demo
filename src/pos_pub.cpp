#include "std_msgs/Float64MultiArray.h" 
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pos_pub");

  ros::NodeHandle n;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  geometry_msgs::PoseStamped current_pose;

  ros::Publisher pos_pub = n.advertise<std_msgs::Float64MultiArray>("pos_pub", 1000); 
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::Float64MultiArray msg;
    current_pose = move_group.getCurrentPose();
    std::vector<double> test_array = {current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z};
    msg.data = test_array;

    ROS_INFO("I have published array data [%f], [%f], [%f]",msg.data[0], msg.data[1], msg.data[2]); 
    pos_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // while (ros::ok())
  // {
  //   std_msgs::Float64MultiArray msg;
  //   std::vector<double> test_array = {0.75, 0.38};
  //   msg.data = test_array;
 
  //   ROS_INFO("I have published array data"); 
  //   chatter_pub.publish(msg);

  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }


  return 0;
}

