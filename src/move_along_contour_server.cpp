#include <ros/ros.h>
#include <std_msgs/String.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <move_demo/MoveAlongContour.h>


const double VERTICAL_STEP = 0.005;
const double RADIAL_STEP = 0.01;

const double GRIPPER_HEIGHT = 0.125;
const double GRIPPER_JOINT_RADIUS = 0.055;

void DownwardMove(double& position_z);
void InwardMove(double& position_x, double& position_y);
void OutwardMove(double& posiiton_x, double& position_y);

bool MoveAlongContour(move_demo::MoveAlongContour::Request &req, move_demo::MoveAlongContour::Response &res)
{
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  geometry_msgs::Pose target_pose3;
  target_pose3.orientation.x = current_pose.pose.orientation.x;
  target_pose3.orientation.y = current_pose.pose.orientation.y;
  target_pose3.orientation.z = current_pose.pose.orientation.z;
  target_pose3.orientation.w = current_pose.pose.orientation.w;
  target_pose3.position.x = current_pose.pose.position.x;
  target_pose3.position.y = current_pose.pose.position.y;
  target_pose3.position.z = current_pose.pose.position.z;

  move_group.setPoseTarget(target_pose3);
  success = true;

  while(success)
  {
    move_group.move();
    ROS_INFO_STREAM(target_pose3.position.x<<" "<<target_pose3.position.y<<" "<<target_pose3.position.z);
    InwardMove(target_pose3.position.x, target_pose3.position.y);
    move_group.setPoseTarget(target_pose3);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(!success)
      OutwardMove(target_pose3.position.x, target_pose3.position.y);

    DownwardMove(target_pose3.position.z);
    if(target_pose3.position.z <= 1.0 + GRIPPER_JOINT_RADIUS)
      break;
    move_group.setPoseTarget(target_pose3);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    while(!success)
    {
      OutwardMove(target_pose3.position.x, target_pose3.position.y);
      move_group.setPoseTarget(target_pose3);
      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
}

  ROS_INFO_STREAM("Step 3 finished");
  res.msg  = "success";
  
  return true;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_along_contour_server");
  ros::NodeHandle n;
  ros::AsyncSpinner async_spinner(4);
  async_spinner.start();
  ros::ServiceServer service = n.advertiseService("move_along_contour", MoveAlongContour);
  ROS_INFO("Step 3 ready. Waiting for request...");
  
  
  ros::waitForShutdown();
  //ros::spin();

  return 0;
}

void DownwardMove(double& position_z)
{
  position_z -= VERTICAL_STEP;
  ROS_INFO_STREAM("next height: " << position_z);
}

void InwardMove(double& position_x, double& position_y)
{
  position_x += RADIAL_STEP;
  position_y += RADIAL_STEP;
  ROS_INFO_STREAM("Inward");
}

void OutwardMove(double& position_x, double& position_y)
{
  position_x -= RADIAL_STEP;
  position_y -= RADIAL_STEP;
  ROS_INFO_STREAM("Outward");
}
