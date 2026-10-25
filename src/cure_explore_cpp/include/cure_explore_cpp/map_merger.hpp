/**
 * @file map_merger.hpp
 * @brief Header for distance-weighted map fusion
 */

#ifndef CURE_EXPLORE_CPP__MAP_MERGER_HPP_
#define CURE_EXPLORE_CPP__MAP_MERGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cure_explore_interfaces/msg/reset_ack.hpp>

#include <mutex>
#include <vector>
#include <unordered_map>
#include <memory>

namespace cure_explore_cpp
{

// Global map parameters
constexpr double GLOBAL_MAP_RESOLUTION = 0.05;  // 5cm per cell
constexpr double GLOBAL_MAP_ORIGIN_X = -50.0;   // Centered 50m-wide global map
constexpr double GLOBAL_MAP_ORIGIN_Y = -50.0;   // Centered 50m-wide global map
constexpr int GLOBAL_MAP_SIZE_X = 2000;         // 50 meters at 5cm resolution
constexpr int GLOBAL_MAP_SIZE_Y = 2000;         // 50 meters at 5cm resolution

// Reset parameters
constexpr double RESET_LOCKOUT_DURATION = 5.0;  // seconds

/**
 * @class MapMerger
 * @brief Multi-robot map fusion with distance-weighted updates
 * 
 * This node merges maps from multiple robots using a distance-weighted
 * fusion approach with visibility filtering and obstacle persistence.
 */
class MapMerger : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @param options Node options
   */
  explicit MapMerger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Parameters
  int robot_count_;
  std::string robot_namespace_;
  std::string global_frame_;
  double update_rate_;
  bool use_visibility_filter_;

  // Map data
  std::vector<int8_t> global_map_data_;      // Binary occupancy (0=free, 100=occupied, -1=unknown)
  std::vector<int8_t> global_map_analog_;    // Analog values for weighted fusion
  std::vector<float> global_map_weights_;    // Fusion weights

  // Caches
  std::unordered_map<int, nav_msgs::msg::OccupancyGrid::SharedPtr> map_data_cache_;
  std::unordered_map<int, geometry_msgs::msg::TransformStamped::SharedPtr> map_tf_cache_;
  std::unordered_map<int, geometry_msgs::msg::TransformStamped::SharedPtr> odom_tf_cache_;

  // Subscriptions
  std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr> map_tf_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr> odom_tf_subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_pub_;
  std::unordered_map<int, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> filtered_map_pubs_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_region_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_region_viz_pub_;
  rclcpp::Publisher<cure_explore_interfaces::msg::ResetAck>::SharedPtr reset_ack_pub_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Thread safety
  std::mutex data_mutex_;

  // Reset state
  bool reset_in_progress_;
  std::chrono::steady_clock::time_point reset_lockout_time_;

  /**
   * @brief Callback for individual robot maps
   * @param msg Occupancy grid message
   * @param robot_id Robot identifier
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, int robot_id);

  /**
   * @brief Callback for synced map->base_link transforms
   * @param msg Transform message
   * @param robot_id Robot identifier
   */
  void mapTfCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg, int robot_id);

  /**
   * @brief Callback for synced odom->base_link transforms
   * @param msg Transform message
   * @param robot_id Robot identifier
   */
  void odomTfCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg, int robot_id);

  /**
   * @brief Callback for reset signal
   * @param msg Empty message
   */
  void resetCallback(const std_msgs::msg::Empty::SharedPtr msg);

  /**
   * @brief Merge a single robot's map into the global map
   * @param robot_id Robot identifier
   */
  void mergeSingleRobotMap(int robot_id);

  /**
   * @brief Check if a cell is visible from the robot's position
   * @param robot_x Robot X coordinate in local grid
   * @param robot_y Robot Y coordinate in local grid
   * @param target_x Target cell X coordinate
   * @param target_y Target cell Y coordinate
   * @param local_data Local map data
   * @param local_width Local map width
   * @param local_height Local map height
   * @return True if target cell is visible
   */
  bool isVisibleFromRobot(
    int robot_x, int robot_y,
    int target_x, int target_y,
    const std::vector<int8_t> & local_data,
    int local_width, int local_height);

  /**
   * @brief Fill isolated unknown cells surrounded by free cells
   */
  void fillIsolatedUnknowns();

  /**
   * @brief Fill unknown cells surrounded by obstacles (gap filling)
   */
  void fillUnknownGapsInObstacles();

  /**
   * @brief Repair small gaps within obstacle walls
   */
  void repairObstacleGaps();

  /**
   * @brief Visualize the merge regions for debugging
   * @param robot_id Robot identifier
   * @param local_map Local map data
   * @param tx X translation
   * @param ty Y translation
   * @param yaw Rotation angle
   */
  void visualizeMergeRegions(
    int robot_id,
    const nav_msgs::msg::OccupancyGrid & local_map,
    double tx, double ty, double yaw);

  /**
   * @brief Publish the global map
   */
  void publishGlobalMap();
};

}  // namespace cure_explore_cpp

#endif  // CURE_EXPLORE_CPP__MAP_MERGER_HPP_
