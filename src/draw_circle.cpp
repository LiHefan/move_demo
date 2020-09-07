#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/Float64MultiArray.h"
//#include <math.h>

const double PI = 3.1416;
const double object_x = 0.45;
const double object_y = 0;
const double GRIPPER_HEIGHT = 0.125;

const double VERTICAL_STEP = 0.01;
const double RADIAL_STEP = 0.02;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_demo", ros::init_options::AnonymousName);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // We will use the :planning_scene_interface:PlanningSceneInterface class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::Pose target_pose;
  tf::Quaternion q;
  double initial_rad = atan2(object_y,object_x);
  q.setRPY(PI,0,-PI/4 + initial_rad);

  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();
  target_pose.position.x = object_x;
  target_pose.position.y = object_y;
  target_pose.position.z = 1.3;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose);
  double centerA = target_pose.position.x;
  double centerB = target_pose.position.y;
  double radius = 0.1;
  
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.1; //0.0
  const double eef_step = 0.01;
  double fraction = 0.0;


  move_group.setPoseTarget(target_pose);
  move_group.move();

  while(target_pose.position.z>1.0+GRIPPER_HEIGHT)
  {
    target_pose.position.z -= VERTICAL_STEP;
    while(fraction<1.0)
    {
      radius += RADIAL_STEP;
      for(double th=0.0; th<6.28;th=th+0.01)
      {
        q.setRPY(PI,0,-PI/4+th);
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        target_pose.position.x = centerA + radius*cos(th);
        target_pose.position.y = centerB + radius*sin(th);
        waypoints.push_back(target_pose);
        fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      }
    }
    my_plan.trajectory_ = trajectory;
    move_group.execute(my_plan);
    ROS_INFO("Next height: %f",target_pose.position.z+VERTICAL_STEP);
  }

  // int maxtries = 100;
  // int attempts = 0;

  // while(fraction < 1.0 && attempts < maxtries)
  // {
  //   fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  //   attempts++;

  //   if(attempts % 10 == 0)
  //     ROS_INFO("Still trying after %d attempts...", attempts);
  // }

  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  ROS_INFO("CurrentZ: %f", current_pose.pose.position.z);
  ROS_INFO("Circle finished.");
  ros::waitForShutdown();
}

