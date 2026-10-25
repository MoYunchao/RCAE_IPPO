// Copyright 2025 SenseBeetle
// SPDX-License-Identifier: Apache-2.0

#ifndef MAP_POSTPROCESSOR__MAP_POSTPROCESSOR_HPP_
#define MAP_POSTPROCESSOR__MAP_POSTPROCESSOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <queue>
#include <atomic>
#include <mutex>
#include <cmath>

namespace map_postprocessor
{

/**
 * @brief Map postprocessor node that removes unreachable free cells
 *
 * This node performs connected component analysis on occupancy grid maps
 * to identify and remove isolated free cells that are unreachable from
 * the robot's current position. These isolated regions are typically
 * noise from laser beams penetrating thin obstacles.
 *
 * Algorithm: Flood-fill (BFS) from robot position to find all reachable
 * free cells. Any free cell not reached is marked as unknown (-1).
 */
class MapPostprocessor : public rclcpp::Node
{
public:
  explicit MapPostprocessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MapPostprocessor() = default;

private:
  /**
   * @brief Callback for raw map data
   * @param msg Incoming occupancy grid from SLAM
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Get robot position in map frame
   * @param x Output x coordinate in map cells
   * @param y Output y coordinate in map cells
   * @return true if position was successfully obtained
   */
  bool getRobotPosition(int & x, int & y);

  /**
   * @brief Perform flood-fill from seed point to find all reachable cells
   * @param data Map data array (will be modified in place)
   * @param width Map width in cells
   * @param height Map height in cells
   * @param seed_x Starting x coordinate
   * @param seed_y Starting y coordinate
   * @param visited Output vector marking visited cells
   */
  void floodFill(
    std::vector<int8_t> & data,
    int width, int height,
    int seed_x, int seed_y,
    std::vector<bool> & visited);

  /**
   * @brief Remove isolated free cells not reachable from robot
   * @param msg Map message to process (modified in place)
   * @return Number of cells cleaned
   */
  size_t removeIsolatedFreeCells(nav_msgs::msg::OccupancyGrid & msg);

  /**
   * @brief Dilate obstacles in the map
   * @param data Map data array (will be modified in place)
   * @param width Map width in cells
   * @param height Map height in cells
   * @param radius Dilation radius in cells
   */
  void dilateObstacles(
    std::vector<int8_t> & data,
    int width, int height,
    int radius);

  /**
   * @brief Compute visibility from robot using ray casting
   *
   * Casts rays in 360 degrees from robot position.
   * Only free cells that can be reached by rays without
   * crossing obstacles are marked as visible.
   *
   * @param data Map data array (read-only for checking obstacles)
   * @param width Map width in cells
   * @param height Map height in cells
   * @param robot_x Robot x position in cells
   * @param robot_y Robot y position in cells
   * @param visible Output vector marking visible cells
   */
  void computeVisibility(
    const std::vector<int8_t> & data,
    int width, int height,
    int robot_x, int robot_y,
    std::vector<bool> & visible);

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string map_frame_;
  std::string base_frame_;
  std::string input_topic_;
  std::string output_topic_;
  int free_threshold_;      // Cells with value <= this are considered free (default: 25)
  int unknown_value_;       // Value for unknown cells (default: -1)
  bool use_robot_position_; // Use robot position as seed (true) or map center (false)
  double min_process_interval_; // Minimum time between processing (seconds)
  int dilation_radius_;     // Obstacle dilation radius in cells (default: 2)
  double ray_angle_step_;   // Ray casting angle step in degrees (default: 0.5)

  // State
  rclcpp::Time last_process_time_;
  std::mutex process_mutex_;

  // Pre-allocated buffers for efficiency
  std::vector<bool> visited_buffer_;
  std::vector<std::pair<int, int>> queue_buffer_;
  std::vector<int8_t> dilated_buffer_;  // Buffer for dilated obstacle map
};

}  // namespace map_postprocessor

#endif  // MAP_POSTPROCESSOR__MAP_POSTPROCESSOR_HPP_
