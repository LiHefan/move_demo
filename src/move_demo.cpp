#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/planning_scene/planning_scene.h>
//#include <moveit/kinematic_constraints/utils.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

const double PI = 3.1416;

const double CONE_HEIGHT = 0.2;
const double CONE_RADIUS = 0.1;
const double BALL_RADIUS = 0.1;

const double OBJECT_X = 0.4;
const double OBJECT_Y = 0.4;

const double VERTICAL_STEP = 0.005;
const double RADIAL_STEP = 0.01;

const double GRIPPER_HEIGHT = 0.125;
const double GRIPPER_JOINT_RADIUS = 0.055; //0.04-0.05

void DownwardMove(double& position_z);
void InwardMove(double& position_x, double& position_y, double object_rad);
void OutwardMove(double& posiiton_x, double& position_y, double object_rad);
void GenerateObjects(moveit_msgs::CollisionObject& ball);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_demo", ros::init_options::AnonymousName);
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4);
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

  // Print current pose
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  ROS_INFO_STREAM("CURRENT POSE: "<<current_pose.pose.orientation.x<<" "<<current_pose.pose.orientation.y<<" "<<current_pose.pose.orientation.z<<" "<<current_pose.pose.orientation.w<<" ");
  
  // /*************Declare PointCloud*******************/
  // ros::Publisher pcl_pub = node_handle.advertise<sensor_msgs::PointCloud2>("pcl_output",100);
  // pcl::PointCloud<pcl::PointXYZ> cloud;
  // sensor_msgs::PointCloud2 output;
  // // Fill in the cloud data
  // cloud.width  = 100;
  // cloud.height = 1;
  // cloud.points.resize(cloud.width * cloud.height);
  // int pcl_counter = 0;


  /*************** Generate Objects ******************/
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject ball;
  GenerateObjects(ball);
  // Create a list of all objects.Put our object into it.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(ball);
  // Add objects to the scene after all objects are in the list.
  planning_scene_interface.addCollisionObjects(collision_objects);

  /************************ Step 1: Move above to the object *************************/
  tf::Quaternion q;
  //q.setRPY(PI/2,PI/4,3*PI/4);
  double object_rad = atan2(OBJECT_Y,OBJECT_X);
  q.setRPY(PI,0,-PI/4+object_rad);
    
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = q.x();
  target_pose1.orientation.y = q.y();
  target_pose1.orientation.z = q.z();
  target_pose1.orientation.w = q.w();
  target_pose1.position.x = OBJECT_X;
  target_pose1.position.y = OBJECT_Y;
  target_pose1.position.z = 1.4;




  move_group.setPoseTarget(target_pose1);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    
  // while(success)
  // {
      
  //   ROS_INFO_STREAM(target_pose1.position.z);
  //   DownwardMove(target_pose1.position.z);
  //   move_group.setPoseTarget(target_pose1);
  //   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // }
    
  // target_pose1.position.z += VERTICAL_STEP;
  // move_group.setPoseTarget(target_pose1);
  move_group.move();
    

  ROS_INFO_STREAM("Step 1 finished");
  ROS_INFO_STREAM("Gripper position: "<< target_pose1.position.z);
    
  /************************* Step 2: Move to the edge ******************************/
  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.x = target_pose1.orientation.x;
  target_pose2.orientation.y = target_pose1.orientation.y;
  target_pose2.orientation.z = target_pose1.orientation.z;
  target_pose2.orientation.w = target_pose1.orientation.w;
  target_pose2.position.x = target_pose1.position.x;
  target_pose2.position.y = target_pose1.position.y;
  target_pose2.position.z = target_pose1.position.z - VERTICAL_STEP;
  move_group.setPoseTarget(target_pose2);
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  while(!success)
  {
    OutwardMove(target_pose2.position.x, target_pose2.position.y,object_rad);
    move_group.setPoseTarget(target_pose2);
    ROS_INFO_STREAM(target_pose2.position.x);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  move_group.move();
  ROS_INFO_STREAM("Step 2 finished");

  /*********************** Step 3: Move along the contour of the object *****************/
  while(success)
  {
    // cloud.points[pcl_counter].x = target_pose2.position.x;
    // cloud.points[pcl_counter].y = target_pose2.position.y;
    // cloud.points[pcl_counter].z = target_pose2.position.z;

    // pcl_counter++;

    move_group.move();
    DownwardMove(target_pose2.position.z);
    if(target_pose2.position.z <= 1.0 + GRIPPER_JOINT_RADIUS)
      break;
    move_group.setPoseTarget(target_pose2);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // if(success)
    // {
    //   InwardMove(target_pose2.position.x, target_pose2.position.y,object_rad);
    //   move_group.setPoseTarget(target_pose2);
    //   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // }
    // while(!success)
    // {
    //   OutwardMove(target_pose2.position.x, target_pose2.position.y,object_rad);
    //   move_group.setPoseTarget(target_pose2);
    //   success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // }
    while(!success)
    {
      OutwardMove(target_pose2.position.x, target_pose2.position.y,object_rad);
      move_group.setPoseTarget(target_pose2);
      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

  
  }

  ROS_INFO_STREAM("Step 3 finished");

  // /*********************************** Visualize data as PointCloud *******************************/
  // pcl::toROSMsg(cloud, output);
  // output.header.frame_id = "odom";
  // ros::Rate loop_rate(100);
  // while (ros::ok())
  // {
  //     pcl_pub.publish(output);
  //     ros::spinOnce();
  //     loop_rate.sleep();
  // }


  ros::waitForShutdown();
    
}


void DownwardMove(double& position_z)
{
  position_z -= VERTICAL_STEP;
  ROS_INFO_STREAM("next height: " << position_z);
}

void InwardMove(double& position_x, double& position_y, double object_rad)
{
  position_x += RADIAL_STEP*cos(object_rad);
  position_y += RADIAL_STEP*sin(object_rad);
  ROS_INFO_STREAM("Inward");
}

void OutwardMove(double& position_x, double& position_y, double object_rad)
{
  position_x -= RADIAL_STEP*cos(object_rad);
  position_y -= RADIAL_STEP*sin(object_rad);
  ROS_INFO_STREAM("Outward");
}

void GenerateObjects(moveit_msgs::CollisionObject& ball)
{
  //ball.header.frame_id = move_group.getPlanningFrame();
  ball.header.frame_id = "world";
  ball.id = "ball1";
  // Objects shape and size
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.SPHERE;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = BALL_RADIUS;
  // Objects position
  geometry_msgs::Pose object_pose1;
  object_pose1.position.x = OBJECT_X;
  object_pose1.position.y = OBJECT_Y;
  object_pose1.position.z = 1.0 + BALL_RADIUS;
  // Pushback shape,size and position
  ball.primitives.push_back(primitive);
  ball.primitive_poses.push_back(object_pose1);
  ball.operation = ball.ADD;
}



