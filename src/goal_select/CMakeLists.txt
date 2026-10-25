cmake_minimum_required(VERSION 3.8)
project(goal_select)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(fuzzy_msgs REQUIRED)
find_package(frontier_msgs REQUIRED)

add_executable(multi_robot_goal_selector_node src/multi_robot_goal_selector_node.cpp)

target_include_directories(multi_robot_goal_selector_node PRIVATE
  include
)

# ✅ 关键修改在这里
ament_target_dependencies(
  multi_robot_goal_selector_node
  rclcpp
  std_msgs
  geometry_msgs
  nav2_msgs
  rclcpp_action
  fuzzy_msgs
  frontier_msgs
)

install(TARGETS multi_robot_goal_selector_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
