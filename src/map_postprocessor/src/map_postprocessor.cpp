// Copyright 2025 SenseBeetle
// SPDX-License-Identifier: Apache-2.0

#include "map_postprocessor/map_postprocessor.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

namespace map_postprocessor
{

MapPostprocessor::MapPostprocessor(const rclcpp::NodeOptions & options)
: Node("map_postprocessor", options)
{
  // Declare parameters
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("input_topic", "/map");
  this->declare_parameter("output_topic", "/map_postprocessed");
  this->declare_parameter("free_threshold", 25);
  this->declare_parameter("unknown_value", -1);
  this->declare_parameter("use_robot_position", true);
  this->declare_parameter("min_process_interval", 0.1);
  this->declare_parameter("dilation_radius", 2);       // Obstacle dilation radius in cells
  this->declare_parameter("ray_angle_step", 0.5);      // Ray angle step in degrees

  // Get parameters
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  free_threshold_ = this->get_parameter("free_threshold").as_int();
  unknown_value_ = this->get_parameter("unknown_value").as_int();
  use_robot_position_ = this->get_parameter("use_robot_position").as_bool();
  min_process_interval_ = this->get_parameter("min_process_interval").as_double();
  dilation_radius_ = this->get_parameter("dilation_radius").as_int();
  ray_angle_step_ = this->get_parameter("ray_angle_step").as_double();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create publisher with same QoS as slam_toolbox
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_qos.reliable();

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, map_qos);

  // Create subscriber
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    input_topic_, map_qos,
    std::bind(&MapPostprocessor::mapCallback, this, std::placeholders::_1));

  last_process_time_ = this->now();

  RCLCPP_INFO(
    this->get_logger(),
    "Map postprocessor initialized: %s -> %s (free_threshold=%d, dilation=%d, ray_step=%.1f)",
    input_topic_.c_str(), output_topic_.c_str(), free_threshold_,
    dilation_radius_, ray_angle_step_);
}

void MapPostprocessor::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(process_mutex_);

  // Rate limiting
  auto now = this->now();
  if ((now - last_process_time_).seconds() < min_process_interval_) {
    return;
  }

  // Make a copy to modify
  auto output_msg = *msg;

  // Process the map
  size_t cleaned_cells = removeIsolatedFreeCells(output_msg);

  // Publish result
  map_pub_->publish(output_msg);
  last_process_time_ = now;

  if (cleaned_cells > 0) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Cleaned %zu isolated free cells from map",
      cleaned_cells);
  }
}

bool MapPostprocessor::getRobotPosition(int & x, int & y)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      map_frame_, base_frame_,
      tf2::TimePointZero,
      std::chrono::milliseconds(100));

    // These will be filled by the caller with map origin info
    x = static_cast<int>(transform.transform.translation.x);
    y = static_cast<int>(transform.transform.translation.y);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "Could not get robot position: %s", ex.what());
    return false;
  }
}

void MapPostprocessor::dilateObstacles(
  std::vector<int8_t> & data,
  int width, int height,
  int radius)
{
  if (radius <= 0) {
    return;
  }

  // 4-connected neighbors for boundary detection
  const int dx4[] = {0, 1, 0, -1};
  const int dy4[] = {1, 0, -1, 0};

  // Step 1: Find obstacle boundary cells (obstacles adjacent to free cells)
  // This is O(W×H) but only done once
  std::vector<std::pair<int, int>> boundary_cells;
  boundary_cells.reserve(width + height);  // Estimate

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int idx = y * width + x;
      int8_t val = data[idx];

      // Only consider obstacle cells
      if (val <= free_threshold_ || val < 0) {
        continue;
      }

      // Check if this obstacle has any free neighbor
      bool has_free_neighbor = false;
      for (int i = 0; i < 4; ++i) {
        int nx = x + dx4[i];
        int ny = y + dy4[i];

        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }

        int nidx = ny * width + nx;
        if (data[nidx] >= 0 && data[nidx] <= free_threshold_) {
          has_free_neighbor = true;
          break;
        }
      }

      if (has_free_neighbor) {
        boundary_cells.emplace_back(x, y);
      }
    }
  }

  // Step 2: Dilate only around boundary cells - O(boundary × R²)
  for (const auto & [bx, by] : boundary_cells) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        // Circular dilation
        if (dx * dx + dy * dy > radius * radius) {
          continue;
        }

        int nx = bx + dx;
        int ny = by + dy;

        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }

        int nidx = ny * width + nx;
        // Only dilate free cells
        if (data[nidx] >= 0 && data[nidx] <= free_threshold_) {
          data[nidx] = 100;  // Mark as dilated obstacle
        }
      }
    }
  }
}

void MapPostprocessor::computeVisibility(
  const std::vector<int8_t> & data,
  int width, int height,
  int robot_x, int robot_y,
  std::vector<bool> & visible)
{
  visible.assign(data.size(), false);

  // Robot position is always visible
  if (robot_x >= 0 && robot_x < width && robot_y >= 0 && robot_y < height) {
    visible[robot_y * width + robot_x] = true;
  }

  // Maximum ray length (diagonal of the map)
  double max_ray_length = std::sqrt(static_cast<double>(width * width + height * height));

  // Cast rays in 360 degrees
  double angle_step_rad = ray_angle_step_ * M_PI / 180.0;
  int num_rays = static_cast<int>(std::ceil(2.0 * M_PI / angle_step_rad));

  for (int r = 0; r < num_rays; ++r) {
    double angle = r * angle_step_rad;
    double dx = std::cos(angle);
    double dy = std::sin(angle);

    // Trace the ray using Bresenham-like stepping
    for (double t = 0; t <= max_ray_length; t += 0.5) {
      int cx = robot_x + static_cast<int>(std::round(t * dx));
      int cy = robot_y + static_cast<int>(std::round(t * dy));

      // Boundary check
      if (cx < 0 || cx >= width || cy < 0 || cy >= height) {
        break;
      }

      int idx = cy * width + cx;
      int8_t val = data[idx];

      // If this cell is an obstacle (dilated), stop the ray
      if (val > free_threshold_ && val >= 0) {
        break;
      }

      // Mark free cells as visible
      if (val >= 0 && val <= free_threshold_) {
        visible[idx] = true;
      }

      // Unknown cells (-1) don't stop the ray, but are not marked visible
    }
  }
}

void MapPostprocessor::floodFill(
  std::vector<int8_t> & data,
  int width, int height,
  int seed_x, int seed_y,
  std::vector<bool> & visited)
{
  // Boundary check for seed
  if (seed_x < 0 || seed_x >= width || seed_y < 0 || seed_y >= height) {
    return;
  }

  // Use pre-allocated queue buffer for efficiency
  // Using a simple vector as a circular queue
  queue_buffer_.clear();
  queue_buffer_.reserve(width * height / 4);  // Reserve reasonable space

  queue_buffer_.emplace_back(seed_x, seed_y);
  size_t front = 0;

  // 4-connected neighbors (faster than 8-connected, sufficient for grid)
  const int dx[] = {0, 1, 0, -1};
  const int dy[] = {1, 0, -1, 0};

  while (front < queue_buffer_.size()) {
    auto [cx, cy] = queue_buffer_[front++];
    int idx = cy * width + cx;

    // Skip if already visited
    if (visited[idx]) {
      continue;
    }
    visited[idx] = true;

    // Check neighbors
    for (int i = 0; i < 4; ++i) {
      int nx = cx + dx[i];
      int ny = cy + dy[i];

      // Boundary check
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }

      int nidx = ny * width + nx;

      // Skip if already visited
      if (visited[nidx]) {
        continue;
      }

      // Only traverse free cells (value >= 0 and <= free_threshold_)
      int8_t val = data[nidx];
      if (val >= 0 && val <= free_threshold_) {
        queue_buffer_.emplace_back(nx, ny);
      }
    }
  }
}

size_t MapPostprocessor::removeIsolatedFreeCells(nav_msgs::msg::OccupancyGrid & msg)
{
  int width = static_cast<int>(msg.info.width);
  int height = static_cast<int>(msg.info.height);
  size_t total_cells = width * height;

  if (total_cells == 0) {
    return 0;
  }

  // Resize visited buffer
  visited_buffer_.assign(total_cells, false);

  // Get seed position
  int seed_x, seed_y;

  if (use_robot_position_) {
    // Try to get robot position from TF
    try {
      auto transform = tf_buffer_->lookupTransform(
        map_frame_, base_frame_,
        tf2::TimePointZero,
        std::chrono::milliseconds(100));

      // Convert world coordinates to map cell coordinates
      double robot_x = transform.transform.translation.x;
      double robot_y = transform.transform.translation.y;

      seed_x = static_cast<int>((robot_x - msg.info.origin.position.x) / msg.info.resolution);
      seed_y = static_cast<int>((robot_y - msg.info.origin.position.y) / msg.info.resolution);

    } catch (const tf2::TransformException &) {
      // Fall back to map center
      seed_x = width / 2;
      seed_y = height / 2;
      RCLCPP_DEBUG(this->get_logger(), "Using map center as seed (TF unavailable)");
    }
  } else {
    // Use map center
    seed_x = width / 2;
    seed_y = height / 2;
  }

  // Clamp seed to map bounds
  seed_x = std::max(0, std::min(width - 1, seed_x));
  seed_y = std::max(0, std::min(height - 1, seed_y));

  // Find a free cell near the seed point (in case seed is on obstacle/unknown)
  // Search in expanding squares around seed
  bool found_free = false;
  int search_radius = 0;
  const int max_search_radius = std::min(width, height) / 2;

  while (!found_free && search_radius <= max_search_radius) {
    for (int dy = -search_radius; dy <= search_radius && !found_free; ++dy) {
      for (int dx = -search_radius; dx <= search_radius && !found_free; ++dx) {
        // Only check boundary of the search square for efficiency
        if (search_radius > 0 &&
          std::abs(dx) != search_radius && std::abs(dy) != search_radius)
        {
          continue;
        }

        int nx = seed_x + dx;
        int ny = seed_y + dy;

        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
          int idx = ny * width + nx;
          int8_t val = msg.data[idx];
          if (val >= 0 && val <= free_threshold_) {
            seed_x = nx;
            seed_y = ny;
            found_free = true;
          }
        }
      }
    }
    ++search_radius;
  }

  if (!found_free) {
    // No free cells in map, nothing to clean
    return 0;
  }

  // Step 1: Dilate obstacles on a working copy
  // This expands obstacles to account for robot footprint
  std::vector<int8_t> working_data = msg.data;
  dilateObstacles(working_data, width, height, dilation_radius_);

  // Step 2: Flood fill from robot position on the DILATED map
  // This finds all cells that are connected to the robot through
  // free space (considering dilated obstacles as barriers)
  visited_buffer_.assign(total_cells, false);
  floodFill(working_data, width, height, seed_x, seed_y, visited_buffer_);

  // Step 3: Expand visited region using frontier-based BFS
  // Only process cells at the boundary of visited region - O(frontier × R)
  const int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};  // 8-connected
  const int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};

  // Find initial frontier: visited cells that have unvisited free neighbors
  std::vector<std::pair<int, int>> frontier;
  frontier.reserve(width + height);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      size_t idx = y * width + x;
      if (!visited_buffer_[idx]) {
        continue;
      }

      // Check if this visited cell has any unvisited originally-free neighbor
      for (int i = 0; i < 8; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];

        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }

        size_t nidx = ny * width + nx;
        if (!visited_buffer_[nidx]) {
          int8_t original_val = msg.data[nidx];
          if (original_val >= 0 && original_val <= free_threshold_) {
            frontier.emplace_back(nx, ny);
            visited_buffer_[nidx] = true;  // Mark immediately to avoid duplicates
          }
        }
      }
    }
  }

  // BFS expansion for remaining passes - O(frontier_size × R)
  for (int pass = 1; pass < dilation_radius_ && !frontier.empty(); ++pass) {
    std::vector<std::pair<int, int>> next_frontier;

    for (const auto & [fx, fy] : frontier) {
      for (int i = 0; i < 8; ++i) {
        int nx = fx + dx[i];
        int ny = fy + dy[i];

        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
          continue;
        }

        size_t nidx = ny * width + nx;
        if (visited_buffer_[nidx]) {
          continue;
        }

        int8_t original_val = msg.data[nidx];
        if (original_val >= 0 && original_val <= free_threshold_) {
          visited_buffer_[nidx] = true;
          next_frontier.emplace_back(nx, ny);
        }
      }
    }

    frontier = std::move(next_frontier);
  }

  // Step 4: Filter out free cells that are not connected
  size_t cleaned = 0;
  for (size_t i = 0; i < total_cells; ++i) {
    int8_t val = msg.data[i];
    if (val >= 0 && val <= free_threshold_ && !visited_buffer_[i]) {
      msg.data[i] = static_cast<int8_t>(unknown_value_);
      ++cleaned;
    }
  }

  return cleaned;
}

}  // namespace map_postprocessor

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(map_postprocessor::MapPostprocessor)
