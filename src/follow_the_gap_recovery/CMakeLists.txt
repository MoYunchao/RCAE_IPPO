cmake_minimum_required(VERSION 3.8)
project(follow_the_gap_recovery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowTheGap.action"
  DEPENDENCIES builtin_interfaces
)

# Build plugin as a shared library
add_library(follow_the_gap_recovery_plugin SHARED
  src/follow_the_gap_recovery.cpp
)

target_include_directories(follow_the_gap_recovery_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(follow_the_gap_recovery_plugin PUBLIC cxx_std_17)

ament_target_dependencies(
  follow_the_gap_recovery_plugin
  "rclcpp"
  "nav2_core"
  "nav2_behaviors"
  "nav2_costmap_2d"
  "nav2_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "nav2_util"
  "pluginlib"
  "tf2"
  "tf2_geometry_msgs"
)

# Link against the generated action interfaces
rosidl_target_interfaces(follow_the_gap_recovery_plugin
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Export plugin
pluginlib_export_plugin_description_file(nav2_core follow_the_gap_recovery_plugin.xml)

# Install plugin library
install(TARGETS follow_the_gap_recovery_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include files
install(DIRECTORY include/
  DESTINATION include/
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

ament_package()
