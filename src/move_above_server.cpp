#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "std_msgs/Float64MultiArray.h"
#include <move_demo/MoveAbove.h>

const double PI = 3.1416;
const double OBJECT_X = 0.75;
const double OBJECT_Y = 0.38;
const double GRIPPER_HEIGHT = 0.125;
const double VERTICAL_STEP = 0.005;

double object_x = 0;
double object_y = 0;

void DownwardMove(double& position_z);

// void chatterCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
// {
//     ROS_INFO("I heard [%f], [%f]", msg->data[0], msg->data[1]);
//     object_x = msg->data[0];
//     object_y = msg->data[1];
//     ROS_INFO("Copy [%f],[%f]",object_x,object_y); 
// }

bool MoveAbove(move_demo::MoveAbove::Request &req, move_demo::MoveAbove::Response &res)
{
  // ros::NodeHandle n;
  // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // ros::spinOnce();

  ros::NodeHandle n;
  boost::shared_ptr<std_msgs::Float64MultiArray const> shared_Ptr;
  shared_Ptr = ros::topic::waitForMessage<std_msgs::Float64MultiArray>("object_position");
  object_x = shared_Ptr->data[0];
  object_y = shared_Ptr->data[1];

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  tf::Quaternion q;
  q.setRPY(PI/2,PI/4,3*PI/4);
    
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();
  target_pose1.position.x = object_x - GRIPPER_HEIGHT*0.7;
  target_pose1.position.y = object_y - GRIPPER_HEIGHT*0.7;
  target_pose1.position.z = 1.45;

  ROS_INFO_STREAM(target_pose1.orientation.x<<" "<<target_pose1.orientation.y<<" "<<target_pose1.orientation.z<<" "<<target_pose1.orientation.w);
  ROS_INFO_STREAM(target_pose1.position.x + GRIPPER_HEIGHT*0.7<<" "<<target_pose1.position.y + GRIPPER_HEIGHT*0.7);


  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
  while(success)
  {
      
    ROS_INFO_STREAM(target_pose1.position.z);
    DownwardMove(target_pose1.position.z);
    move_group.setPoseTarget(target_pose1);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
    
  target_pose1.position.z += VERTICAL_STEP;
  move_group.setPoseTarget(target_pose1);
  move_group.move();
    

  ROS_INFO_STREAM("Step 1 finished");
  ROS_INFO_STREAM("Gripper position: "<< target_pose1.position.z);
  res.msg = "success";

  return true;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_above_server");
  ros::NodeHandle n;
  ros::AsyncSpinner async_spinner(4);
  async_spinner.start();
  ros::ServiceServer service = n.advertiseService("move_above", MoveAbove);
  ROS_INFO("Step 1 ready. Waiting for request...");


  ros::waitForShutdown();
  //ros::spin();

  return 0;
}

void DownwardMove(double& position_z)
{
  position_z -= VERTICAL_STEP;
  ROS_INFO_STREAM("next height: " << position_z);
}