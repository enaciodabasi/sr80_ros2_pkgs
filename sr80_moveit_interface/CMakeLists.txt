cmake_minimum_required(VERSION 3.8)
project(sr80_moveit_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sr80_custom_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Boost
  control_msgs
  geometry_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  sr80_custom_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

#add_library(action_server SHARED src/sr80_moveit_action_server.cpp)
#target_include_directories(action_server PRIVATE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#ament_target_dependencies(action_server ${THIS_PACKAGE_INCLUDE_DEPENDS})
#
#add_library(pose_action_client SHARED
#  src/sr80_moveit_pose_action_client.cpp
#)
#target_include_directories(pose_action_client PRIVATE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#
#ament_target_dependencies(pose_action_client ${THIS_PACKAGE_INCLUDE_DEPENDS})
#
#add_library(joint_space_action_client SHARED
#  src/sr80_moveit_joint_space_action_client.cpp
#)
#target_include_directories(joint_space_action_client PRIVATE
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#ament_target_dependencies(joint_space_action_client ${THIS_PACKAGE_INCLUDE_DEPENDS})
#
#
#rclcpp_components_register_node(action_server PLUGIN "SR80MoveitActionServer" EXECUTABLE sr80_action_server)
#rclcpp_components_register_node(pose_action_client PLUGIN "PoseGoalClient" EXECUTABLE sr80_pose_action_client)
#rclcpp_components_register_node(joint_space_action_client PLUGIN "JointSpaceClient" EXECUTABLE sr80_joint_space_action_client)

#add_executable(sr80_moveit_action_server src/sr80_moveit_action_server.cpp)
#ament_target_dependencies(sr80_moveit_action_server ${THIS_PACKAGE_INCLUDE_DEPENDS})

#add_executable(moveitcpp_node src/moveitcpp_node.cpp)
#ament_target_dependencies(moveitcpp_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(action_server  
  src/moveitcpp_action_server.cpp)
ament_target_dependencies(action_server ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(action_client
  src/moveitcpp_action_client.cpp
)
ament_target_dependencies(action_client ${THIS_PACKAGE_INCLUDE_DEPENDS})


add_executable(moveit_cpp src/moveit_cpp.cpp)
ament_target_dependencies(moveit_cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})

#install(
#  TARGETS
#  action_server
#  pose_action_client
#  joint_space_action_client  
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)

install(
  TARGETS
  action_server
  action_client
  moveit_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  include
  src
  DESTINATION
  share/${PROJECT_NAME}
)

ament_package()
