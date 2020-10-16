cmake_minimum_required(VERSION 3.0.2)
project(move_demo)

find_package(catkin REQUIRED COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    roscpp
    rospy
    std_msgs
    message_generation
)

add_service_files(
  FILES
  AddObject.srv
  MoveAbove.srv
  MoveToEdge.srv
  MoveAlongContour.srv
  DrawCircle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
 
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)



include_directories(
   ${catkin_INCLUDE_DIRS} 	
)
 
link_directories(
  ${catkin_LIB_DIRS} 
)
 
 
#lets moveit
add_executable(move_demo src/move_demo.cpp)
target_link_libraries(move_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(add_object_server src/add_object_server.cpp)
target_link_libraries(add_object_server ${catkin_LIBRARIES})
add_dependencies(add_object_server move_demo_gencpp)

add_executable(move_above_server src/move_above_server.cpp)
target_link_libraries(move_above_server ${catkin_LIBRARIES})
add_dependencies(move_above_server move_demo_gencpp)

add_executable(move_to_edge_server src/move_to_edge_server.cpp)
target_link_libraries(move_to_edge_server ${catkin_LIBRARIES})
add_dependencies(move_to_edge_server move_demo_gencpp)

add_executable(move_along_contour_server src/move_along_contour_server.cpp)
target_link_libraries(move_along_contour_server ${catkin_LIBRARIES})
add_dependencies(move_along_contour_server move_demo_gencpp)

add_executable(object_position_pub src/object_position_pub.cpp)
target_link_libraries(object_position_pub ${catkin_LIBRARIES})

add_executable(gripper_position_pub src/gripper_position_pub.cpp)
target_link_libraries(gripper_position_pub ${catkin_LIBRARIES})

add_executable(tns_pub src/tns_pub.cpp)
target_link_libraries(tns_pub ${catkin_LIBRARIES})

add_executable(sub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})

add_executable(draw_circle src/draw_circle.cpp)
target_link_libraries(draw_circle ${catkin_LIBRARIES})

add_executable(draw_circle_server src/draw_circle_server.cpp)
target_link_libraries(draw_circle_server ${catkin_LIBRARIES})


install(TARGETS move_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS scripts/state.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

