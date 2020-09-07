#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_arm_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

  // Collision Checking
  // ^^^^^^^^^^^^^^^^^^
  //
  // Self-collision checking
  // ~~~~~~~~~~~~~~~~~~~~~~~
  //
  // The first thing we will do is check whether the robot in its
  // current state is in *self-collision*, i.e. whether the current
  // configuration of the robot would result in the robot's parts
  // hitting each other. To do this, we will construct a
  // :collision_detection_struct:`CollisionRequest` object and a
  // :collision_detection_struct:`CollisionResult` object and pass them
  // into the collision checking function. Note that the result of
  // whether the robot is in self-collision or not is contained within
  // the result. Self collision checking uses an *unpadded* version of
  // the robot, i.e. it directly uses the collision meshes provided in
  // the URDF with no extra padding added on.

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");


  // Now, we can get contact information for any collisions that might
  // have happened at a given configuration of the Panda arm. We can ask
  // for contact information by filling in the appropriate field in the
  // collision request and specifying the maximum number of contacts to
  // be returned as a large number.

    collision_request.contacts = true;
    collision_request.max_contacts = 1000;

  //

    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
    collision_detection::CollisionResult::ContactMap::const_iterator it;
    for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
    {
      ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
    }

  // Modifying the Allowed Collision Matrix
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // The :collision_detection_class:`AllowedCollisionMatrix` (ACM)
  // provides a mechanism to tell the collision world to ignore
  // collisions between certain object: both parts of the robot and
  // objects in the world. We can tell the collision checker to ignore
  // all collisions between the links reported above, i.e. even though
  // the links are actually in collision, the collision checker will
  // ignore those collisions and return not in collision for this
  // particular state of the robot.
  //
  // Note also in this example how we are making copies of both the
  // allowed collision matrix and the current state and passing them in
  // to the collision checking function.

    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    robot_state::RobotState copied_state = planning_scene.getCurrentState();

    collision_detection::CollisionResult::ContactMap::const_iterator it2;
    for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
    {
      acm.setEntry(it2->first.first, it2->first.second, true);
    }
  // Full Collision Checking
  // ~~~~~~~~~~~~~~~~~~~~~~~
  //
  // While we have been checking for self-collisions, we can use the
  // checkCollision functions instead which will check for both
  // self-collisions and for collisions with the environment (which is
  // currently empty).  This is the set of collision checking
  // functions that you will use most often in a planner. Note that
  // collision checks with the environment will use the padded version
  // of the robot. Padding helps in keeping the robot further away
  // from obstacles in the environment.*/
    collision_result.clear();
    planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
    ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");




}

    while(success)
    {
      move_group.move();
      target_pose2.position.z -= VERTICAL_STEP;
      ROS_INFO_STREAM("next height: " << target_pose2.position.z);
      move_group.setPoseTarget(target_pose2);
      success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if(target_pose2.position.z <= 1.0 + GRIPPER_HEIGHT)
      {
        break;
      }

      //In- or Outward moving
      while(success)
      {
        target_pose2.position.x += RADIAL_STEP;
        target_pose2.position.y += RADIAL_STEP;
        move_group.setPoseTarget(target_pose2);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Inward");
      }

      while(!success) 
      {
        target_pose2.position.x -= RADIAL_STEP;
        target_pose2.position.y -= RADIAL_STEP;
        move_group.setPoseTarget(target_pose2);
        success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_STREAM("Outward");
      }
      ROS_INFO_STREAM("Downward");

    }