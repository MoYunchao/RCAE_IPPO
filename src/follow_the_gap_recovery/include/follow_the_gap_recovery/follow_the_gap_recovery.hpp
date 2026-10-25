#ifndef FOLLOW_THE_GAP_RECOVERY__FOLLOW_THE_GAP_RECOVERY_HPP_
#define FOLLOW_THE_GAP_RECOVERY__FOLLOW_THE_GAP_RECOVERY_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_behaviors/timed_behavior.hpp"
#include "follow_the_gap_recovery/action/follow_the_gap.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace follow_the_gap_recovery
{

using FollowTheGapAction = follow_the_gap_recovery::action::FollowTheGap;

/**
 * @class FollowTheGapRecovery
 * @brief A recovery behavior that uses the "follow the gap" algorithm
 *        to navigate the robot towards the largest gap in laser scan data.
 *        This is designed for omnidirectional robots that can move in any direction.
 *        The recovery stops when the closest obstacle is > 0.2m away.
 */
class FollowTheGapRecovery : public nav2_behaviors::TimedBehavior<FollowTheGapAction>
{
public:
  using Status = nav2_behaviors::Status;

  FollowTheGapRecovery();
  ~FollowTheGapRecovery() override = default;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  Status onRun(const std::shared_ptr<const FollowTheGapAction::Goal> command) override;

  /**
   * @brief Configuration of behavior
   */
  void onConfigure() override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  Status onCycleUpdate() override;

protected:
  /**
   * @brief Laser scan callback
   * @param scan Laser scan message
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

  /**
   * @brief Find the largest gap in the laser scan
   * @param scan Laser scan data
   * @param gap_start Index of gap start
   * @param gap_end Index of gap end
   * @return True if a valid gap is found
   */
  bool findLargestGap(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    size_t & gap_start,
    size_t & gap_end);

  /**
   * @brief Get the closest obstacle distance from laser scan
   * @param scan Laser scan data
   * @return Minimum distance to obstacle
   */
  double getClosestObstacleDistance(const sensor_msgs::msg::LaserScan::SharedPtr & scan);

  /**
   * @brief Calculate velocity command towards the gap
   * @param scan Laser scan data
   * @param gap_start Index of gap start
   * @param gap_end Index of gap end
   * @return Twist command
   */
  geometry_msgs::msg::Twist calculateGapVelocity(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan,
    size_t gap_start,
    size_t gap_end);

  /**
   * @brief Calculate velocity to escape perpendicular to nearest wall
   * @param scan Laser scan data
   * @return Twist command moving away from wall
   */
  geometry_msgs::msg::Twist calculateWallEscapeVelocity(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan);

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  // Latest scan data
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex scan_mutex_;

  // Parameters
  double min_obstacle_distance_;  // Minimum distance to stop recovery (default: 0.2m)
  double max_linear_velocity_;    // Maximum linear velocity (default: 0.5 m/s)
  double min_gap_distance_;       // Minimum distance to consider as gap (default: 0.5m)
  int min_gap_width_;            // Minimum gap width in scan points (default: 10)
  double bubble_radius_;         // Safety bubble radius around obstacles (default: 0.25m)

  // Feedback
  FollowTheGapAction::Feedback::SharedPtr feedback_;

  // State
  bool scan_received_;
  rclcpp::Duration time_allowance_{0, 0};
};

}  // namespace follow_the_gap_recovery

#endif  // FOLLOW_THE_GAP_RECOVERY__FOLLOW_THE_GAP_RECOVERY_HPP_
