#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/Float64MultiArray.h"
#include <move_demo/DrawCircle.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

const double PI = 3.1416;
const double OBJECT_X = 0.35;
const double OBJECT_Y = 0;
const double GRIPPER_HEIGHT = 0.125;
const double BALL_RADIUS = 0.1;

const double VERTICAL_STEP = 0.01;
const double RADIAL_STEP = 0.02;

bool DrawCircle(move_demo::DrawCircle::Request &req, move_demo::DrawCircle::Response &res)
{
  ros::NodeHandle node_handle;
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // We will use the :planning_scene_interface:PlanningSceneInterface class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  /*************Declare PointCloud*******************/
  ros::Publisher pcl_pub = node_handle.advertise<sensor_msgs::PointCloud2>("pcl_output",100);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  sensor_msgs::PointCloud2 output;
  // Fill in the cloud data
  cloud.width  = 12000;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);
  int pcl_counter = 0;


  geometry_msgs::Pose target_pose;
  tf::Quaternion q;
  double initial_rad = atan2(OBJECT_Y,OBJECT_X);
  q.setRPY(PI,0,-PI/4 + initial_rad);
  
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  target_pose.orientation.w = q.w();
  target_pose.position.x = OBJECT_X;
  target_pose.position.y = OBJECT_Y;
  target_pose.position.z = 1.3;

  //std::vector<geometry_msgs::Pose> waypoints;
  //waypoints.push_back(target_pose);
  double centerA = target_pose.position.x;
  double centerB = target_pose.position.y;
  double radius = 0.1;
  
  std::vector<geometry_msgs::Pose> waypoints;
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0; //0.0
  const double eef_step = 0.01;
  double fraction = 0.0;

  move_group.setPoseTarget(target_pose);
  move_group.move();
  
  int round_counter = 1;

  while(target_pose.position.z>1.13)
  {
    
    radius = 0.1;
    waypoints.clear();
    fraction = 0.0;
    while(fraction<1.0)
    { 
      waypoints.clear();
      current_pose = move_group.getCurrentPose();
      target_pose.position.x = current_pose.pose.position.x;
      target_pose.position.y = current_pose.pose.position.y;
      target_pose.position.z = current_pose.pose.position.z;
      target_pose.orientation.x = current_pose.pose.orientation.x;
      target_pose.orientation.y = current_pose.pose.orientation.y;
      target_pose.orientation.z = current_pose.pose.orientation.z;
      target_pose.orientation.w = current_pose.pose.orientation.w;
      waypoints.push_back(target_pose);
      target_pose.position.z -= VERTICAL_STEP;
      if(round_counter==1)
      {
        for(double th=0.0; th<1.57;th=th+0.01)
        { 
          q.setRPY(PI,0,-PI/4+th);
          target_pose.orientation.x = q.x();
          target_pose.orientation.y = q.y();
          target_pose.orientation.z = q.z();
          target_pose.orientation.w = q.w();
          target_pose.position.x = centerA + radius*cos(PI+th);
          target_pose.position.y = centerB + radius*sin(PI+th);
          waypoints.push_back(target_pose);
        }        
      }
      else
      {
        for(double th=1.57; th>0;th=th-0.01)
        { 
          q.setRPY(PI,0,-PI/4+th);
          target_pose.orientation.x = q.x();
          target_pose.orientation.y = q.y();
          target_pose.orientation.z = q.z();
          target_pose.orientation.w = q.w();
          target_pose.position.x = centerA + radius*cos(PI+th);
          target_pose.position.y = centerB + radius*sin(PI+th);
          waypoints.push_back(target_pose);
        }  
      }
      
    fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    }
    my_plan.trajectory_ = trajectory;
    move_group.execute(my_plan);
    round_counter = round_counter*(-1);
    for(int i=0;i<waypoints.size();i++)
    {
      cloud.points[pcl_counter].x = waypoints[i].position.x;
      cloud.points[pcl_counter].y = waypoints[i].position.y;
      cloud.points[pcl_counter].z = waypoints[i].position.z;
      pcl_counter++;
    }
    ROS_INFO("Next height: %f",target_pose.position.z - VERTICAL_STEP);
    ROS_INFO("Fraction = %f",fraction);

  }

  ROS_INFO("CurrentZ: %f", current_pose.pose.position.z);
  ROS_INFO("Circle finished.");
  /*********************************** Visualize data as PointCloud *******************************/
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "odom";
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
      pcl_pub.publish(output);
      ros::spinOnce();
      loop_rate.sleep();
  }
  /*************************************************************************************************/
  ROS_INFO_STREAM("Step 4 finished");
  res.msg = "success";

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_above_server");
  ros::NodeHandle n;
  ros::AsyncSpinner async_spinner(4);
  async_spinner.start();
  ros::ServiceServer service = n.advertiseService("draw_circle", DrawCircle);
  ROS_INFO("Step 4 ready. Waiting for request...");


  ros::waitForShutdown();
  //ros::spin();

  return 0;
}