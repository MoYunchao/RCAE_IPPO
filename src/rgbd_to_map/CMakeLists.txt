cmake_minimum_required(VERSION 3.8)
project(rgbd_to_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(frontier_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common)
find_package(semantic_grid_msgs)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(frontier_detector src/frontier_detector.cpp)
add_executable(rgbd_processor_node src/rgbd_processor_node.cpp)
add_executable(camera_pointcloud_to_laserscan_node src/camera_pointcloud_to_laserscan_node.cpp)
add_executable(scan_mapping_node src/scan_mapping_node.cpp)
add_executable(laserscan_frame_transformer src/laserscan_frame_transformer.cpp)

ament_target_dependencies(frontier_detector rclcpp nav_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs frontier_msgs semantic_grid_msgs)
ament_target_dependencies(rgbd_processor_node rclcpp sensor_msgs std_msgs cv_bridge pcl_conversions tf2 tf2_ros tf2_sensor_msgs tf2_geometry_msgs)
ament_target_dependencies(camera_pointcloud_to_laserscan_node rclcpp sensor_msgs std_msgs tf2_ros tf2 tf2_sensor_msgs pcl_conversions)
ament_target_dependencies(scan_mapping_node rclcpp sensor_msgs nav_msgs tf2 tf2_ros geometry_msgs)
ament_target_dependencies(laserscan_frame_transformer rclcpp sensor_msgs)

target_include_directories(scan_mapping_node PRIVATE
  ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(rgbd_processor_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(camera_pointcloud_to_laserscan_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  frontier_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rgbd_processor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS camera_pointcloud_to_laserscan_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS scan_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS laserscan_frame_transformer
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
