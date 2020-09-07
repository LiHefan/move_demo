#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "std_msgs/Float64MultiArray.h" 

#include <move_demo/AddObject.h>

const double PI = 3.1416;
const double BALL_RADIUS = 0.1;
const double OBJECT_X = 0.75;
const double OBJECT_Y = 0.38;
const double GRIPPER_HEIGHT = 0.125;


bool AddBall(move_demo::AddObject::Request &req, move_demo::AddObject::Response &res)
{
  static const std::string PLANNING_GROUP = "panda_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // We will use the :planning_scene_interface:PlanningSceneInterface class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Define a collision object ROS message.
  moveit_msgs::CollisionObject ball;
  ball.header.frame_id = move_group.getPlanningFrame();
  ball.id = "ball1";
  // Objects shape and size
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = BALL_RADIUS;
  // Objects position
  geometry_msgs::Pose object_pose1;
  object_pose1.orientation.x = 1.0;
  object_pose1.position.x = OBJECT_X;
  object_pose1.position.y = OBJECT_Y;
  object_pose1.position.z = 1.0 + BALL_RADIUS;
  // Pushback shape,size and position
  ball.primitives.push_back(primitive);
  ball.primitive_poses.push_back(object_pose1);
  ball.operation = ball.ADD;
  ROS_INFO("Ball created.");
  // Create a list of all objects.Put our object into it.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(ball);
  // Add objects to the scene after all objects are in the list.
  planning_scene_interface.addCollisionObjects(collision_objects);
  ROS_INFO("Ball added.");

  res.msg = "success";
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_object_server");
  ros::NodeHandle n;
  ros::AsyncSpinner async_spinner(4);
  async_spinner.start();
  ros::ServiceServer service = n.advertiseService("add_object", AddBall);
  ROS_INFO("Ready to add object.");
  
  
  ros::waitForShutdown();
  //ros::spin();

  return 0;
}


