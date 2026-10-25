cmake_minimum_required(VERSION 3.8)
project(fuzzy_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(frontier_msgs REQUIRED)
find_package(fuzzy_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(fuzzy_decision_node
  src/fuzzy_decision_node.cpp
)

add_executable(linear_decision_node
  src/linear_decision_node.cpp
)

add_executable(greedy_decision_node
  src/greedy_decision_node.cpp
)

ament_target_dependencies(fuzzy_decision_node
  rclcpp
  frontier_msgs
  fuzzy_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(linear_decision_node
  rclcpp
  frontier_msgs
  fuzzy_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

ament_target_dependencies(greedy_decision_node
  rclcpp
  frontier_msgs
  fuzzy_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
)

target_include_directories(fuzzy_decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(linear_decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(greedy_decision_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接库
target_link_libraries(fuzzy_decision_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}      
  ${frontier_msgs_TARGETS}
  ${fuzzy_msgs_TARGETS}
)

target_link_libraries(linear_decision_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${frontier_msgs_TARGETS}
  ${fuzzy_msgs_TARGETS}
)

target_link_libraries(greedy_decision_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${frontier_msgs_TARGETS}
  ${fuzzy_msgs_TARGETS}
)

# 安装目标
install(TARGETS fuzzy_decision_node linear_decision_node greedy_decision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY rviz2/
  DESTINATION share/${PROJECT_NAME}/rviz2
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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

ament_export_dependencies(rclcpp std_msgs frontier_msgs fuzzy_msgs)
ament_export_include_directories(include)

ament_package()
