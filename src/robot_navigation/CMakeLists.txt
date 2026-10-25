cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ${CMAKE_EXPORT_COMPILE_COMMANDS})
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(follow_the_gap_recovery REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# Build BT plugin for FollowTheGap
add_library(follow_the_gap_action_bt_node SHARED
  src/follow_the_gap_action_bt_node.cpp
)

target_include_directories(follow_the_gap_action_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(follow_the_gap_action_bt_node
  nav2_behavior_tree
  follow_the_gap_recovery
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
)

# Install BT plugin library
install(TARGETS follow_the_gap_action_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python scripts
install(PROGRAMS
  scripts/odom_to_tf.py
  scripts/robot_lifecycle_manager.py
  scripts/initial_spin.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
    params
    rviz
    launch
    behavior_trees
  )
