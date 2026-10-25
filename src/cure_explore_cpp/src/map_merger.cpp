/**
 * @file map_merger.cpp
 * @brief Implementation of distance-weighted map fusion
 */

#include "cure_explore_cpp/map_merger.hpp"
#include <tf2/utils.h>
#include <chrono>

namespace cure_explore_cpp
{

MapMerger::MapMerger(const rclcpp::NodeOptions & options)
: Node("map_merger",
       rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)),
  reset_in_progress_(false)
{
  // Get parameters
  robot_count_ = this->get_parameter_or("robot_count", 3);
  robot_namespace_ = this->get_parameter_or("robot_namespace", std::string("robot_"));
  global_frame_ = this->get_parameter_or("global_frame", std::string("global_map"));
  update_rate_ = this->get_parameter_or("update_rate", 1.0);
  use_visibility_filter_ = this->get_parameter_or("use_visibility_filter", true);

  // Initialize global map
  global_map_data_.resize(GLOBAL_MAP_SIZE_X * GLOBAL_MAP_SIZE_Y, -1);    // Binary map
  global_map_analog_.resize(GLOBAL_MAP_SIZE_X * GLOBAL_MAP_SIZE_Y, -1);  // Analog map for decay
  global_map_weights_.resize(GLOBAL_MAP_SIZE_X * GLOBAL_MAP_SIZE_Y, 0.0f);

  // Create publishers
  global_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_map", 10);
  local_region_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/map_merge_local_region", 10);
  global_region_viz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/map_merge_global_region", 10);
  reset_ack_pub_ = this->create_publisher<cure_explore_interfaces::msg::ResetAck>(
    "/cure_explore/internal_reset_ack", 10);

  // QoS for transient local topics
  rclcpp::QoS qos_transient(10);
  qos_transient.durability(rclcpp::DurabilityPolicy::TransientLocal);
  qos_transient.reliability(rclcpp::ReliabilityPolicy::Reliable);

  // Create subscriptions for each robot
  for (int i = 0; i < robot_count_; i++) {
    int robot_id = i;
    std::string map_topic = "/" + robot_namespace_ + std::to_string(robot_id) + "/map_postprocessed";
    std::string map_tf_topic = "/" + robot_namespace_ + std::to_string(robot_id) + "/map_synced_tf";
    std::string odom_tf_topic = "/" + robot_namespace_ + std::to_string(robot_id) + "/odom_synced_tf";

    // Map subscription
    auto map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, 10,
      [this, robot_id](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->mapCallback(msg, robot_id);
      });
    map_subs_.push_back(map_sub);

    // Map->base_link transform subscription
    auto map_tf_sub = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      map_tf_topic, qos_transient,
      [this, robot_id](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        this->mapTfCallback(msg, robot_id);
      });
    map_tf_subs_.push_back(map_tf_sub);

    // Odom->base_link transform subscription
    auto odom_tf_sub = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      odom_tf_topic, qos_transient,
      [this, robot_id](const geometry_msgs::msg::TransformStamped::SharedPtr msg) {
        this->odomTfCallback(msg, robot_id);
      });
    odom_tf_subs_.push_back(odom_tf_sub);

    // Filtered local map publisher (for visibility debugging)
    std::string filtered_map_topic = "/" + robot_namespace_ + std::to_string(robot_id) + "/filtered_map";
    filtered_map_pubs_[robot_id] = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      filtered_map_topic, 10);
  }

  // Reset subscription
  reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/cure_explore/reset", 10,
    std::bind(&MapMerger::resetCallback, this, std::placeholders::_1));

  // TF2 setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Timer for publishing global map
  double period = 1.0 / update_rate_;
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&MapMerger::publishGlobalMap, this));

  RCLCPP_INFO(this->get_logger(),
              "Map Merger (C++) initialized for %d robots", robot_count_);
  RCLCPP_INFO(this->get_logger(),
              "  Visibility filter: %s", use_visibility_filter_ ? "ENABLED" : "disabled");
}

void MapMerger::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, int robot_id)
{
  // Check reset lockout
  if (reset_in_progress_) {
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - reset_lockout_time_).count();
    if (elapsed < RESET_LOCKOUT_DURATION) {
      return;
    } else {
      reset_in_progress_ = false;
      RCLCPP_INFO(this->get_logger(), "Reset lockout ended, accepting new maps");
    }
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_data_cache_[robot_id] = msg;
  }


  mergeSingleRobotMap(robot_id);
}

void MapMerger::mapTfCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg, int robot_id)
{
  if (reset_in_progress_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  map_tf_cache_[robot_id] = msg;

  RCLCPP_DEBUG(this->get_logger(),
               "Received synced map->base_link for robot_%d: pos=[%.3f, %.3f]",
               robot_id, msg->transform.translation.x, msg->transform.translation.y);
}

void MapMerger::odomTfCallback(const geometry_msgs::msg::TransformStamped::SharedPtr msg, int robot_id)
{
  if (reset_in_progress_) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  odom_tf_cache_[robot_id] = msg;

  RCLCPP_DEBUG(this->get_logger(),
               "Received synced odom->base_link for robot_%d: pos=[%.3f, %.3f]",
               robot_id, msg->transform.translation.x, msg->transform.translation.y);
}

void MapMerger::resetCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
{
  RCLCPP_INFO(this->get_logger(), "Received reset signal, starting reset procedure");

  // Set reset lockout
  reset_in_progress_ = true;
  reset_lockout_time_ = std::chrono::steady_clock::now();

  // Clear caches
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_data_cache_.clear();
    map_tf_cache_.clear();
    odom_tf_cache_.clear();
  }
  RCLCPP_INFO(this->get_logger(), "  Cleared map and TF caches");

  // Reset global map (both binary and analog)
  std::fill(global_map_data_.begin(), global_map_data_.end(), -1);
  std::fill(global_map_analog_.begin(), global_map_analog_.end(), -1);
  std::fill(global_map_weights_.begin(), global_map_weights_.end(), 0.0f);
  RCLCPP_INFO(this->get_logger(), "  Reset global maps (binary and analog) to unknown state");

  // Publish empty map
  publishGlobalMap();
  RCLCPP_INFO(this->get_logger(), "  Published empty global map");

  // Send reset acknowledgment
  cure_explore_interfaces::msg::ResetAck ack;
  ack.source = "map_merger";
  ack.success = true;
  ack.message = "Map merger reset complete (lockout " +
                std::to_string(RESET_LOCKOUT_DURATION) + "s)";
  reset_ack_pub_->publish(ack);

  RCLCPP_INFO(this->get_logger(), "Map merger reset complete, sent acknowledgment");
}

bool MapMerger::isVisibleFromRobot(
  int robot_x, int robot_y,
  int target_x, int target_y,
  const std::vector<int8_t> & local_data,
  int local_width, int local_height)
{
  // Bresenham's line algorithm for ray casting
  // Returns true if the line from robot to target doesn't pass through obstacles

  int dx = std::abs(target_x - robot_x);
  int dy = std::abs(target_y - robot_y);
  int sx = (robot_x < target_x) ? 1 : -1;
  int sy = (robot_y < target_y) ? 1 : -1;
  int err = dx - dy;

  int x = robot_x;
  int y = robot_y;

  // Threshold for obstacle detection (occupied cells have value >= 50)
  constexpr int8_t OBSTACLE_THRESHOLD = 50;

  while (true) {
    // Check if we've reached the target (or one cell before)
    // We stop one cell before to allow the target cell itself to be visible
    if (std::abs(x - target_x) <= 1 && std::abs(y - target_y) <= 1) {
      return true;  // Visible - reached target without hitting obstacle
    }

    // Bounds check
    if (x < 0 || x >= local_width || y < 0 || y >= local_height) {
      return false;  // Out of bounds
    }

    // Check for obstacle at current position (skip the starting robot position)
    if (x != robot_x || y != robot_y) {
      int8_t cell_val = local_data[y * local_width + x];
      if (cell_val >= OBSTACLE_THRESHOLD) {
        return false;  // Hit an obstacle - not visible
      }
    }

    // Move to next cell along the line
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

void MapMerger::mergeSingleRobotMap(int robot_id)
{
  const auto start_time = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (map_data_cache_.find(robot_id) == map_data_cache_.end()) {
    return;
  }

  auto local_map = map_data_cache_[robot_id];

  try {
    // 1. Get TF: global_frame -> robot_{id}/map
    std::string local_map_frame = robot_namespace_ + std::to_string(robot_id) + "/map";

    // 直接尝试获取TF变换，不做可用性检查
    auto transform_map = tf_buffer_->lookupTransform(
      global_frame_, local_map_frame, tf2::TimePointZero);

    // 2. Get synced map->base_link transform
    geometry_msgs::msg::TransformStamped map_to_base;
    if (map_tf_cache_.find(robot_id) != map_tf_cache_.end()) {
      map_to_base = *map_tf_cache_[robot_id];
    } else {
      // Fallback: try to get from TF
      std::string robot_base_frame = robot_namespace_ + std::to_string(robot_id) + "/base_link";
      try {
        map_to_base = tf_buffer_->lookupTransform(
          local_map_frame, robot_base_frame, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
        return;
      }
    }

    // 3. Extract transform parameters
    double tx = transform_map.transform.translation.x;
    double ty = transform_map.transform.translation.y;
    double qz = transform_map.transform.rotation.z;
    double qw = transform_map.transform.rotation.w;
    double yaw = 2.0 * std::atan2(qz, qw);
    double cos_yaw = std::cos(yaw);
    double sin_yaw = std::sin(yaw);

    // Robot position in map frame
    double r_map_x = map_to_base.transform.translation.x;
    double r_map_y = map_to_base.transform.translation.y;

    // 4. Get local map parameters
    int local_width = local_map->info.width;
    int local_height = local_map->info.height;
    double local_res = local_map->info.resolution;
    double local_origin_x = local_map->info.origin.position.x;
    double local_origin_y = local_map->info.origin.position.y;

    // Convert robot position to local grid coordinates (for visibility check)
    int robot_grid_x = static_cast<int>((r_map_x - local_origin_x) / local_res);
    int robot_grid_y = static_cast<int>((r_map_y - local_origin_y) / local_res);

    // Copy local map data for visibility check
    std::vector<int8_t> local_data(local_map->data.begin(), local_map->data.end());

    // Filtered map data for visualization (only visible cells)
    std::vector<int8_t> filtered_data(local_width * local_height, -1);

    // 5. Merge each cell
    int merged_count = 0;
    int known_cells = 0;
    int visibility_rejected = 0;

    for (int y_local = 0; y_local < local_height; y_local++) {
      for (int x_local = 0; x_local < local_width; x_local++) {
        int8_t local_val = local_map->data[y_local * local_width + x_local];

        // Skip unknown cells
        if (local_val == -1) {
          continue;
        }
        known_cells++;

        // Visibility check: only merge cells visible from robot position
        if (use_visibility_filter_) {
          if (!isVisibleFromRobot(robot_grid_x, robot_grid_y,
                                   x_local, y_local,
                                   local_data, local_width, local_height)) {
            visibility_rejected++;
            continue;
          }
        }

        // Mark as visible in filtered map
        filtered_data[y_local * local_width + x_local] = local_val;

        // Convert local grid to local world coordinates (use cell center)
        double px_local = x_local * local_res + local_res / 2.0 + local_origin_x;
        double py_local = y_local * local_res + local_res / 2.0 + local_origin_y;

        // Apply TF transform to global coordinates
        double wx = tx + px_local * cos_yaw - py_local * sin_yaw;
        double wy = ty + px_local * sin_yaw + py_local * cos_yaw;

        // Convert to global grid coordinates with proper rounding
        int gx = static_cast<int>(std::round((wx - GLOBAL_MAP_ORIGIN_X) / GLOBAL_MAP_RESOLUTION));
        int gy = static_cast<int>(std::round((wy - GLOBAL_MAP_ORIGIN_Y) / GLOBAL_MAP_RESOLUTION));

        // Bounds check
        if (gx < 0 || gx >= GLOBAL_MAP_SIZE_X || gy < 0 || gy >= GLOBAL_MAP_SIZE_Y) {
          continue;
        }

        merged_count++;

        // Determine weight based on observation type (asymmetric weighting)
        // Reduced ratio for faster obstacle decay: 1.2:1 instead of 2:1
        // This allows dynamic obstacles to be cleared in 2-3 observations
        constexpr float OBSTACLE_WEIGHT = 1.2f;  // Slightly higher weight for obstacles
        constexpr float FREE_WEIGHT = 1.0f;      // Standard weight for free space
        constexpr int8_t OBSTACLE_TO_FREE_THRESHOLD = 30;  // Below this value, convert to free (0)

        float new_weight = (local_val >= 50) ? OBSTACLE_WEIGHT : FREE_WEIGHT;

        // For obstacles, also update neighboring cells to handle rounding errors
        // This prevents thin obstacles from "disappearing" due to coordinate transform rounding
        int expand_radius = (local_val >= 50) ? 1 : 0;  // Expand obstacles by 1 cell

        for (int dy = -expand_radius; dy <= expand_radius; dy++) {
          for (int dx = -expand_radius; dx <= expand_radius; dx++) {
            int nx = gx + dx;
            int ny = gy + dy;

            // Bounds check
            if (nx < 0 || nx >= GLOBAL_MAP_SIZE_X || ny < 0 || ny >= GLOBAL_MAP_SIZE_Y) {
              continue;
            }

            int nidx = ny * GLOBAL_MAP_SIZE_X + nx;
            float n_old_weight = global_map_weights_[nidx];
            int8_t n_old_val = global_map_data_[nidx];

            if (n_old_val == -1) {
              // Unknown -> directly set both maps
              // Analog map: store actual value for decay calculation
              global_map_analog_[nidx] = local_val;
              global_map_weights_[nidx] = new_weight;

              // Binary map: immediately binarize
              // >= 50 -> 100 (occupied), < 50 -> 0 (free)
              global_map_data_[nidx] = (local_val >= 50) ? 100 : 0;
            } else {
              // Get analog value for weighted fusion
              int8_t analog_old_val = global_map_analog_[nidx];
              if (analog_old_val == -1) {
                analog_old_val = n_old_val;  // Fallback to binary value
              }

              // Asymmetric weighted fusion on analog map
              float total_weight = n_old_weight + new_weight;
              double fused_val = (analog_old_val * n_old_weight + local_val * new_weight) / total_weight;

              // Store the analog fused value (0-100) for gradual decay
              int8_t rounded_val = static_cast<int8_t>(std::round(fused_val));

              // If value drops below threshold, convert to free space immediately
              if (rounded_val < OBSTACLE_TO_FREE_THRESHOLD && rounded_val > 0) {
                rounded_val = 0;
              }

              global_map_analog_[nidx] = rounded_val;
              global_map_weights_[nidx] = std::min(total_weight, 10.0f);

              // Binary map: immediately binarize the analog value
              // >= 50 -> 100 (occupied), < 50 -> 0 (free)
              global_map_data_[nidx] = (rounded_val >= 50) ? 100 : 0;
            }
          }
        }
      }
    }

    // Publish filtered local map for visualization
    if (use_visibility_filter_ && filtered_map_pubs_.count(robot_id) > 0) {
      nav_msgs::msg::OccupancyGrid filtered_map;
      filtered_map.header.stamp = this->get_clock()->now();
      filtered_map.header.frame_id = robot_namespace_ + std::to_string(robot_id) + "/map";
      filtered_map.info = local_map->info;
      filtered_map.data = filtered_data;
      filtered_map_pubs_[robot_id]->publish(filtered_map);
    }

    if (known_cells > 0) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Robot_%d: merge complete, known=%d, merged=%d, visibility_rejected=%d",
                   robot_id, known_cells, merged_count, visibility_rejected);
    }

    // Visualize merge regions
    visualizeMergeRegions(robot_id, *local_map, tx, ty, yaw);

    // Post-processing: fill isolated unknowns and repair obstacle gaps
    if (merged_count > 0) {
      fillIsolatedUnknowns();
      fillUnknownGapsInObstacles();
      repairObstacleGaps();
    }

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(),
                "Could not transform for robot_%d: %s. Will retry on next map update.", 
                robot_id, ex.what());
    // 不return，让函数自然结束，等待下次地图更新再尝试
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Error in map merge for robot_%d: %s", robot_id, e.what());
  }

  const auto elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time).count();
  if (elapsed_ms >= 150.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "[PERF/MAP_MERGER] mergeSingleRobotMap slow: robot_%d %.1f ms",
      robot_id, elapsed_ms);
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[PERF/MAP_MERGER] mergeSingleRobotMap: robot_%d %.1f ms",
      robot_id, elapsed_ms);
  }
}

void MapMerger::fillIsolatedUnknowns()
{
  // Fill isolated unknown cells surrounded by free cells
  // Similar to scipy.ndimage.convolve with 3x3 kernel

  std::vector<std::pair<int, int>> cells_to_fill;

  for (int y = 1; y < GLOBAL_MAP_SIZE_Y - 1; y++) {
    for (int x = 1; x < GLOBAL_MAP_SIZE_X - 1; x++) {
      int idx = y * GLOBAL_MAP_SIZE_X + x;

      // Only check unknown cells
      if (global_map_data_[idx] != -1) {
        continue;
      }

      // Count free neighbors (8-connectivity)
      int free_count = 0;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          int nidx = (y + dy) * GLOBAL_MAP_SIZE_X + (x + dx);
          if (global_map_data_[nidx] == 0) {
            free_count++;
          }
        }
      }

      // If all 8 neighbors are free, mark for filling
      if (free_count == 8) {
        cells_to_fill.push_back({x, y});
      }
    }
  }

  // Fill the cells (update both binary and analog maps)
  for (const auto & [x, y] : cells_to_fill) {
    int idx = y * GLOBAL_MAP_SIZE_X + x;
    global_map_data_[idx] = 0;    // Binary: free
    global_map_analog_[idx] = 0;  // Analog: free
  }

  if (!cells_to_fill.empty()) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Filled %zu isolated unknown cells", cells_to_fill.size());
  }
}

void MapMerger::fillUnknownGapsInObstacles()
{
  // Fill unknown cells that are surrounded by obstacles
  // This fixes gaps in obstacle walls caused by rounding/sampling errors
  
  std::vector<std::pair<int, int>> cells_to_fill;

  for (int y = 1; y < GLOBAL_MAP_SIZE_Y - 1; y++) {
    for (int x = 1; x < GLOBAL_MAP_SIZE_X - 1; x++) {
      int idx = y * GLOBAL_MAP_SIZE_X + x;

      // Only check unknown cells (value -1)
      if (global_map_data_[idx] != -1) {
        continue;
      }

      // Count obstacle neighbors (8-connectivity)
      int obstacle_count = 0;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          int nidx = (y + dy) * GLOBAL_MAP_SIZE_X + (x + dx);
          if (global_map_data_[nidx] >= 80) {
            obstacle_count++;
          }
        }
      }

      // If at least 6 out of 8 neighbors are obstacles, fill as obstacle
      // This is more aggressive than the all-8 requirement for free cells
      if (obstacle_count >= 6) {
        cells_to_fill.push_back({x, y});
      }
    }
  }

  // Fill the gaps as obstacles (update both binary and analog maps)
  for (const auto & [x, y] : cells_to_fill) {
    int idx = y * GLOBAL_MAP_SIZE_X + x;
    global_map_data_[idx] = 100;    // Binary: occupied
    global_map_analog_[idx] = 100;  // Analog: occupied
  }

  if (!cells_to_fill.empty()) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Filled %zu unknown gaps in obstacles", cells_to_fill.size());
  }
}

void MapMerger::repairObstacleGaps()
{
  // Fill isolated free cells ONLY if they are completely surrounded by obstacles
  // This fixes gaps caused by rounding errors while allowing dynamic obstacles to be removed
  
  std::vector<std::pair<int, int>> cells_to_fill;

  for (int y = 1; y < GLOBAL_MAP_SIZE_Y - 1; y++) {
    for (int x = 1; x < GLOBAL_MAP_SIZE_X - 1; x++) {
      int idx = y * GLOBAL_MAP_SIZE_X + x;

      // Only check free cells (value 0)
      if (global_map_data_[idx] != 0) {
        continue;
      }

      // Count obstacle neighbors (8-connectivity)
      int obstacle_count = 0;
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          int nidx = (y + dy) * GLOBAL_MAP_SIZE_X + (x + dx);
          if (global_map_data_[nidx] >= 80) {
            obstacle_count++;
          }
        }
      }

      // Only fill if ALL 8 neighbors are obstacles (completely surrounded)
      // This is more conservative - only fixes obvious rounding errors
      if (obstacle_count == 8) {
        cells_to_fill.push_back({x, y});
      }
    }
  }

  // Fill the gaps (update both binary and analog maps)
  for (const auto & [x, y] : cells_to_fill) {
    int idx = y * GLOBAL_MAP_SIZE_X + x;
    global_map_data_[idx] = 100;    // Binary: occupied
    global_map_analog_[idx] = 100;  // Analog: occupied
  }

  if (!cells_to_fill.empty()) {
    RCLCPP_DEBUG(this->get_logger(),
                 "Repaired %zu obstacle gaps", cells_to_fill.size());
  }
}

void MapMerger::visualizeMergeRegions(
  int robot_id,
  const nav_msgs::msg::OccupancyGrid & local_map,
  double tx, double ty, double yaw)
{
  // Color based on robot ID
  float r, g, b;
  switch (robot_id) {
    case 0: r = 1.0f; g = 0.0f; b = 0.0f; break;  // Red
    case 1: r = 0.0f; g = 1.0f; b = 0.0f; break;  // Green
    case 2: r = 0.0f; g = 0.0f; b = 1.0f; break;  // Blue
    default: r = 1.0f; g = 0.0f; b = 1.0f; break;  // Magenta
  }

  // Local map boundary
  double min_x = local_map.info.origin.position.x;
  double min_y = local_map.info.origin.position.y;
  double max_x = min_x + local_map.info.width * local_map.info.resolution;
  double max_y = min_y + local_map.info.height * local_map.info.resolution;

  std::vector<std::pair<double, double>> corners = {
    {min_x, min_y}, {max_x, min_y}, {max_x, max_y}, {min_x, max_y}, {min_x, min_y}
  };

  // 1. Local region marker
  visualization_msgs::msg::Marker local_marker;
  local_marker.header.stamp = this->get_clock()->now();
  local_marker.header.frame_id = robot_namespace_ + std::to_string(robot_id) + "/map";
  local_marker.ns = "local_map_boundary";
  local_marker.id = robot_id;
  local_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  local_marker.action = visualization_msgs::msg::Marker::ADD;
  local_marker.pose.orientation.w = 1.0;
  local_marker.scale.x = 0.05;
  local_marker.color.r = r;
  local_marker.color.g = g;
  local_marker.color.b = b;
  local_marker.color.a = 0.8f;

  for (const auto & [lx, ly] : corners) {
    geometry_msgs::msg::Point p;
    p.x = lx;
    p.y = ly;
    p.z = 0.1;
    local_marker.points.push_back(p);
  }

  local_region_viz_pub_->publish(local_marker);

  // 2. Global region marker
  visualization_msgs::msg::Marker global_marker;
  global_marker.header.stamp = this->get_clock()->now();
  global_marker.header.frame_id = global_frame_;
  global_marker.ns = "global_merge_boundary";
  global_marker.id = robot_id;
  global_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  global_marker.action = visualization_msgs::msg::Marker::ADD;
  global_marker.pose.orientation.w = 1.0;
  global_marker.scale.x = 0.08;
  global_marker.color.r = r;
  global_marker.color.g = g;
  global_marker.color.b = b;
  global_marker.color.a = 1.0f;

  double cos_yaw = std::cos(yaw);
  double sin_yaw = std::sin(yaw);

  for (const auto & [lx, ly] : corners) {
    double gx = tx + lx * cos_yaw - ly * sin_yaw;
    double gy = ty + lx * sin_yaw + ly * cos_yaw;

    geometry_msgs::msg::Point p;
    p.x = gx;
    p.y = gy;
    p.z = 0.2;
    global_marker.points.push_back(p);
  }

  global_region_viz_pub_->publish(global_marker);
}

void MapMerger::publishGlobalMap()
{
  const auto start_time = std::chrono::steady_clock::now();
  nav_msgs::msg::OccupancyGrid global_map_msg;
  global_map_msg.header.stamp = this->get_clock()->now();
  global_map_msg.header.frame_id = global_frame_;
  global_map_msg.info.resolution = GLOBAL_MAP_RESOLUTION;
  global_map_msg.info.width = GLOBAL_MAP_SIZE_X;
  global_map_msg.info.height = GLOBAL_MAP_SIZE_Y;
  global_map_msg.info.origin.position.x = GLOBAL_MAP_ORIGIN_X;
  global_map_msg.info.origin.position.y = GLOBAL_MAP_ORIGIN_Y;
  global_map_msg.info.origin.orientation.w = 1.0;
  global_map_msg.data = global_map_data_;

  global_map_pub_->publish(global_map_msg);
  const auto elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start_time).count();
  if (elapsed_ms >= 50.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "[PERF/MAP_MERGER] publishGlobalMap slow: %.1f ms, global_map=%dx%d",
      elapsed_ms, GLOBAL_MAP_SIZE_X, GLOBAL_MAP_SIZE_Y);
  } else {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "[PERF/MAP_MERGER] publishGlobalMap: %.1f ms, global_map=%dx%d",
      elapsed_ms, GLOBAL_MAP_SIZE_X, GLOBAL_MAP_SIZE_Y);
  }
}

}  // namespace cure_explore_cpp
