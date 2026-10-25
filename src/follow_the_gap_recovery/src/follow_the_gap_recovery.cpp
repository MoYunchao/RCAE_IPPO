#include "follow_the_gap_recovery/follow_the_gap_recovery.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace follow_the_gap_recovery
{

FollowTheGapRecovery::FollowTheGapRecovery()
: scan_received_(false)
{
  feedback_ = std::make_shared<FollowTheGapAction::Feedback>();
}

void FollowTheGapRecovery::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  // Declare and get parameters
  node->declare_parameter(behavior_name_ + ".min_obstacle_distance", 0.2);
  node->declare_parameter(behavior_name_ + ".max_linear_velocity", 0.5);
  node->declare_parameter(behavior_name_ + ".min_gap_distance", 0.5);
  node->declare_parameter(behavior_name_ + ".min_gap_width", 10);
  node->declare_parameter(behavior_name_ + ".bubble_radius", 0.25);

  node->get_parameter(behavior_name_ + ".min_obstacle_distance", min_obstacle_distance_);
  node->get_parameter(behavior_name_ + ".max_linear_velocity", max_linear_velocity_);
  node->get_parameter(behavior_name_ + ".min_gap_distance", min_gap_distance_);
  node->get_parameter(behavior_name_ + ".min_gap_width", min_gap_width_);
  node->get_parameter(behavior_name_ + ".bubble_radius", bubble_radius_);

  RCLCPP_INFO(
    logger_,
    "Configured %s with parameters - min_obstacle_distance: %.2f, max_linear_velocity: %.2f, "
    "min_gap_distance: %.2f, min_gap_width: %d, bubble_radius: %.2f",
    behavior_name_.c_str(), min_obstacle_distance_, max_linear_velocity_,
    min_gap_distance_, min_gap_width_, bubble_radius_);

  // Create scan subscriber
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    std::bind(&FollowTheGapRecovery::scanCallback, this, std::placeholders::_1));
}

FollowTheGapRecovery::Status FollowTheGapRecovery::onRun(
  const std::shared_ptr<const FollowTheGapAction::Goal> command)
{
  RCLCPP_INFO(logger_, "Starting Follow the Gap Recovery with time allowance: %.2f s",
    rclcpp::Duration(command->time_allowance).seconds());

  // Store time allowance from goal
  time_allowance_ = command->time_allowance;

  // Reset state
  scan_received_ = false;

  return Status::SUCCEEDED;
}

FollowTheGapRecovery::Status FollowTheGapRecovery::onCycleUpdate()
{
  // Wait for scan data
  if (!scan_received_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000,
      "Waiting for laser scan data...");
    return Status::RUNNING;
  }

  // Get latest scan
  sensor_msgs::msg::LaserScan::SharedPtr scan;
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan = latest_scan_;
  }

  if (!scan) {
    return Status::RUNNING;
  }

  // Check if we should stop recovery
  double closest_distance = getClosestObstacleDistance(scan);

  // Update feedback
  feedback_->current_closest_distance = closest_distance;
  action_server_->publish_feedback(feedback_);

  RCLCPP_INFO(
    logger_,
    "Cycle Update: Closest obstacle distance: %.2f m (Threshold: %.2f m)", 
    closest_distance, min_obstacle_distance_);

  if (closest_distance > min_obstacle_distance_) {
    RCLCPP_INFO(
      logger_,
      "Recovery successful! Closest obstacle is %.2f m away (threshold: %.2f m)",
      closest_distance, min_obstacle_distance_);

    // Stop the robot
    geometry_msgs::msg::Twist stop_cmd;
    vel_pub_->publish(stop_cmd);
    return Status::SUCCEEDED;
  }

  // Find largest gap
  size_t gap_start, gap_end;
  if (!findLargestGap(scan, gap_start, gap_end)) {
    RCLCPP_WARN(
      logger_,
      "No valid gap found (min_gap_dist: %.2f, min_gap_width: %d), escaping perpendicular to wall...",
      min_gap_distance_, min_gap_width_);

    // Escape perpendicular to nearest wall
    geometry_msgs::msg::Twist escape_cmd = calculateWallEscapeVelocity(scan);
    vel_pub_->publish(escape_cmd);
    return Status::RUNNING;
  }

  // Calculate gap details for logging
  double gap_start_angle = scan->angle_min + gap_start * scan->angle_increment;
  double gap_end_angle = scan->angle_min + gap_end * scan->angle_increment;
  double gap_middle_angle = (gap_start_angle + gap_end_angle) / 2.0;
  size_t gap_width = gap_end - gap_start + 1;

  RCLCPP_INFO(
    logger_,
    "Best Gap Found: Indices [%zu, %zu], Angles [%.2f, %.2f] rad (Middle: %.2f rad), Width: %zu points",
    gap_start, gap_end, gap_start_angle, gap_end_angle, gap_middle_angle, gap_width);

  // Calculate and publish velocity command
  geometry_msgs::msg::Twist cmd_vel = calculateGapVelocity(scan, gap_start, gap_end);

  RCLCPP_INFO(
    logger_,
    "Commanding Velocity: linear(x=%.2f, y=%.2f), angular(z=%.2f)",
    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  vel_pub_->publish(cmd_vel);

  return Status::RUNNING;
}

void FollowTheGapRecovery::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  latest_scan_ = scan;
  scan_received_ = true;
}

bool FollowTheGapRecovery::findLargestGap(
  const sensor_msgs::msg::LaserScan::SharedPtr & scan,
  size_t & gap_start,
  size_t & gap_end)
{
  // 1. Create a copy of ranges for Safety Bubble preprocessing
  std::vector<float> ranges = scan->ranges;
  size_t count = ranges.size();

  // 2. Apply Safety Bubble - inflate obstacles to push gap edges away from walls
  // This prevents the robot from selecting paths too close to obstacles
  for (size_t i = 0; i < count; ++i) {
    float r = scan->ranges[i];

    // If this point is an obstacle (distance less than threshold and valid)
    if (std::isfinite(r) && r < min_gap_distance_) {
      // Calculate the angular range to cover based on bubble radius
      // Using geometry: angle = atan2(bubble_radius, distance)
      double angle_to_cover = std::atan2(bubble_radius_, r);
      int indices_to_cover = static_cast<int>(std::ceil(angle_to_cover / scan->angle_increment));

      // Mark surrounding points as obstacles (set to 0.0)
      int start_idx = std::max(0, static_cast<int>(i) - indices_to_cover);
      int end_idx = std::min(static_cast<int>(count) - 1, static_cast<int>(i) + indices_to_cover);

      for (int j = start_idx; j <= end_idx; ++j) {
        ranges[j] = 0.0f;  // Mark as obstacle/invalid
      }
    }
  }

  // 3. Find the largest gap in the processed ranges
  size_t max_gap_size = 0;
  size_t current_gap_start = 0;
  size_t current_gap_size = 0;
  bool in_gap = false;

  for (size_t i = 0; i < count; ++i) {
    float range = ranges[i];

    // Check if this point is in a gap (must be > threshold AND not masked by bubble)
    if (std::isfinite(range) && range >= min_gap_distance_ && range > 0.01f) {
      if (!in_gap) {
        // Start of a new gap
        current_gap_start = i;
        current_gap_size = 1;
        in_gap = true;
      } else {
        // Continue current gap
        current_gap_size++;
      }
    } else {
      if (in_gap) {
        // End of current gap - check if it's the largest
        if (current_gap_size > max_gap_size &&
            static_cast<int>(current_gap_size) >= min_gap_width_) {
          max_gap_size = current_gap_size;
          gap_start = current_gap_start;
          gap_end = i - 1;
        }
        in_gap = false;
      }
    }
  }

  // Check if gap extends to the end of scan
  if (in_gap && current_gap_size > max_gap_size &&
      static_cast<int>(current_gap_size) >= min_gap_width_) {
    max_gap_size = current_gap_size;
    gap_start = current_gap_start;
    gap_end = count - 1;
  }

  return max_gap_size > 0;
}

double FollowTheGapRecovery::getClosestObstacleDistance(
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  double min_distance = std::numeric_limits<double>::max();

  for (const auto & range : scan->ranges) {
    if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
      min_distance = std::min(min_distance, static_cast<double>(range));
    }
  }

  return min_distance;
}

geometry_msgs::msg::Twist FollowTheGapRecovery::calculateGapVelocity(
  const sensor_msgs::msg::LaserScan::SharedPtr & scan,
  size_t gap_start,
  size_t gap_end)
{
  geometry_msgs::msg::Twist cmd_vel;

  // 1. Calculate the middle of the gap
  size_t gap_middle = (gap_start + gap_end) / 2;

  // Convert gap middle index to angle
  double gap_angle = scan->angle_min + gap_middle * scan->angle_increment;

  // Normalize angle to [-pi, pi]
  while (gap_angle > M_PI) gap_angle -= 2.0 * M_PI;
  while (gap_angle < -M_PI) gap_angle += 2.0 * M_PI;

  RCLCPP_INFO(logger_, "Gap angle: %.2f rad (%.1f deg)", gap_angle, gap_angle * 180.0 / M_PI);

  // 2. Holonomic movement strategy: move directly towards the gap
  // For omnidirectional robots, we can move in any direction without rotating first
  // This is much faster and safer than rotate-then-move

  // Decompose velocity into robot frame (x: forward, y: left)
  // Gap in front (0 rad): x = speed, y = 0
  // Gap on left (π/2 rad): x = 0, y = speed
  // Gap behind (π rad): x = -speed, y = 0
  // Gap on right (-π/2 rad): x = 0, y = -speed
  cmd_vel.linear.x = max_linear_velocity_ * std::cos(gap_angle);
  cmd_vel.linear.y = max_linear_velocity_ * std::sin(gap_angle);

  // 3. Optional: add gentle rotation to align robot heading towards the gap
  // This makes subsequent navigation easier once recovery succeeds
  // P-controller: angular velocity proportional to angle error
  double max_angular_vel = 1.0;  // rad/s
  cmd_vel.angular.z = std::clamp(gap_angle * 1.0, -max_angular_vel, max_angular_vel);

  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;

  RCLCPP_INFO(logger_, "Holonomic Escape: x=%.2f, y=%.2f, w=%.2f",
              cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  return cmd_vel;
}

geometry_msgs::msg::Twist FollowTheGapRecovery::calculateWallEscapeVelocity(
  const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  geometry_msgs::msg::Twist cmd_vel;

  // Find the closest obstacle point
  size_t closest_idx = 0;
  double min_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    float range = scan->ranges[i];
    if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
      if (range < min_distance) {
        min_distance = range;
        closest_idx = i;
      }
    }
  }

  // Calculate angle to closest obstacle
  double obstacle_angle = scan->angle_min + closest_idx * scan->angle_increment;

  // Normalize angle to [-pi, pi]
  while (obstacle_angle > M_PI) obstacle_angle -= 2.0 * M_PI;
  while (obstacle_angle < -M_PI) obstacle_angle += 2.0 * M_PI;

  // Move perpendicular to wall = opposite direction of obstacle
  // Add 180 degrees (π radians) to move away from wall
  double escape_angle = obstacle_angle + M_PI;
  
  // Normalize escape angle
  while (escape_angle > M_PI) escape_angle -= 2.0 * M_PI;
  while (escape_angle < -M_PI) escape_angle += 2.0 * M_PI;

  RCLCPP_INFO(logger_, 
    "Wall Escape: Closest obstacle at %.2f m, angle %.2f rad (%.1f deg). Escaping at %.2f rad (%.1f deg)",
    min_distance, obstacle_angle, obstacle_angle * 180.0 / M_PI,
    escape_angle, escape_angle * 180.0 / M_PI);

  // Calculate holonomic velocity to move away from wall
  double escape_velocity = max_linear_velocity_ * 0.8;  // Use 80% of max speed for safety
  cmd_vel.linear.x = escape_velocity * std::cos(escape_angle);
  cmd_vel.linear.y = escape_velocity * std::sin(escape_angle);

  // Add rotation to align heading away from wall
  double max_angular_vel = 1.5;  // rad/s
  cmd_vel.angular.z = std::clamp(escape_angle * 1.2, -max_angular_vel, max_angular_vel);

  cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = 0.0;
  cmd_vel.angular.y = 0.0;

  RCLCPP_INFO(logger_, "Wall Escape Command: x=%.2f, y=%.2f, w=%.2f",
              cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);

  return cmd_vel;
}

}  // namespace follow_the_gap_recovery

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(follow_the_gap_recovery::FollowTheGapRecovery, nav2_core::Behavior)
