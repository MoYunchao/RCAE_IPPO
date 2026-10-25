cmake_minimum_required(VERSION 3.5)
project(cure_explore_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SemanticObject.msg"
  "msg/SemanticMap.msg"
  "msg/FrontierCluster.msg"
  "msg/FrontierArray.msg"
  "msg/CostInfo.msg"
  "msg/FrontierCost.msg"
  "msg/RobotPose.msg"
  "msg/ResetCommand.msg"
  "msg/ResetAck.msg"
  "srv/ValidateGoal.srv"
  "srv/ComputeFrontierCosts.srv"
  "srv/SetRobotPoses.srv"
  "srv/SystemReset.srv"
  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
  LIBRARY_NAME ${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
