#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <move_demo/MoveToEdge.h>


const double VERTICAL_STEP = 0.005;
const double RADIAL_STEP = 0.01;

const double GRIPPER_HEIGHT = 0.125;
const double GRIPPER_JOINT_RADIUS = 0.055;

void OutwardMove(double& posiiton_x, double& position_y);

bool MoveToEdge(move_demo::MoveToEdge::Request &req, move_demo::MoveToEdge::Response &res)
{
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = current_pose.pose.orientation.x;
  target_pose2.orientation.y = current_pose.pose.orientation.y;
  target_pose2.orientation.z = current_pose.pose.orientation.z;
  target_pose2.orientation.w = current_pose.pose.orientation.w;
  target_pose2.position.x = current_pose.pose.position.x;
  target_pose2.position.y = current_pose.pose.position.y;
  target_pose2.position.z = current_pose.pose.position.z - VERTICAL_STEP;

  move_group.setPoseTarget(target_pose2);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  while(!success)
  {
    OutwardMove(target_pose2.position.x, target_pose2.position.y);
    move_group.setPoseTarget(target_pose2);
    ROS_INFO_STREAM(target_pose2.position.x);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  move_group.move();
  
  ROS_INFO_STREAM("Step 2 finished");
  res.msg  = "success";
  
  return true;

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_edge_server");
  ros::NodeHandle n;
  ros::AsyncSpinner async_spinner(4);
  async_spinner.start();
  ros::ServiceServer service = n.advertiseService("move_to_edge", MoveToEdge);
  ROS_INFO("Step 2 ready. Waiting for request...");
  
  
  ros::waitForShutdown();
  //ros::spin();

  return 0;
}

void OutwardMove(double& position_x, double& position_y)
{
  position_x -= RADIAL_STEP;
  position_y -= RADIAL_STEP;
  ROS_INFO_STREAM("Outward");
}

