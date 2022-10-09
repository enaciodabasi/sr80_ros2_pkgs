cmake_minimum_required(VERSION 3.8)
project(sr80_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/PoseGoalRequest.msg"
  "msg/PoseGoalFeedback.msg"
  "msg/PoseGoalResult.msg"
  "msg/JointSpaceGoalRequest.msg"
  "msg/JointSpaceGoalFeedback.msg"
  "msg/JointSpaceGoalResult.msg"
  "msg/DoubleArray.msg"
  "msg/ServoGoalRequest.msg"
)

set(action_files
  "action/PoseGoal.action"
  "action/JointSpaceGoal.action"
  "action/ServoGoal.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs action_msgs geometry_msgs control_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
