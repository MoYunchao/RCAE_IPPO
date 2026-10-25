#include <chrono>
#include <cmath>
#include <memory>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class BatteryStateSim : public rclcpp::Node
{
public:
  BatteryStateSim()
  : Node("battery_state_sim")
  {
    // ===== 参数 =====
    declare_parameter("frame_id", "base_link");
    declare_parameter("update_rate_hz", 2.0);

    declare_parameter("design_capacity_ah", 10.0);
    declare_parameter("initial_charge_ratio", 1.0);
    declare_parameter("reset_charge_ratio", 1.0);
    declare_parameter("failure_charge_ratio", 0.0);

    declare_parameter("camera_current", 0.06);
    declare_parameter("lidar_current", 0.06);
    declare_parameter("compute_current_gain", 2.0);
    declare_parameter("communication_current_gain", 1.0);
    declare_parameter("max_bandwidth_mbps", 10.0);
    declare_parameter("idle_current", 0.04);

    declare_parameter("kv", 8.0);   // A / (m/s)
    declare_parameter("kw", 1.2);   // A / (rad/s)

    get_parameter("frame_id", frame_id_);
    get_parameter("update_rate_hz", update_rate_hz_);
    get_parameter("design_capacity_ah", design_capacity_ah_);
    get_parameter("initial_charge_ratio", initial_charge_ratio_);
    get_parameter("reset_charge_ratio", reset_charge_ratio_);
    get_parameter("failure_charge_ratio", failure_charge_ratio_);

    get_parameter("camera_current", camera_current_);
    get_parameter("lidar_current", lidar_current_);
    get_parameter("compute_current_gain", compute_current_gain_);
    get_parameter("communication_current_gain", communication_current_gain_);
    get_parameter("max_bandwidth_mbps", max_bandwidth_mbps_);
    get_parameter("idle_current", idle_current_);

    get_parameter("kv", kv_);
    get_parameter("kw", kw_);

    initial_charge_ratio_ = std::clamp(initial_charge_ratio_, 0.0, 1.0);
    reset_charge_ratio_ = std::clamp(reset_charge_ratio_, 0.0, 1.0);
    failure_charge_ratio_ = std::clamp(failure_charge_ratio_, 0.0, 1.0);
    current_ah_ = design_capacity_ah_ * initial_charge_ratio_;

    // ===== Publisher =====
    battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>(
      "battery_state", 10);

    // ===== Subscriber =====
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&BatteryStateSim::cmdVelCallback, this, std::placeholders::_1));
    cpu_sub_ = create_subscription<std_msgs::msg::Float64>(
      "cpu_state", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        cpu_usage_ = std::clamp(msg->data, 0.0, 1.0);
      });
    bandwidth_sub_ = create_subscription<std_msgs::msg::Float64>(
      "network_state", 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        bandwidth_mbps_ = std::clamp(msg->data, 0.0, max_bandwidth_mbps_);
      });

    reset_battery_srv_ = create_service<std_srvs::srv::Trigger>(
      "reset_battery",
      std::bind(
        &BatteryStateSim::resetBatteryCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    set_discharge_enabled_srv_ = create_service<std_srvs::srv::SetBool>(
      "set_discharge_enabled",
      std::bind(
        &BatteryStateSim::setDischargeEnabledCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    // ===== Timer =====
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / update_rate_hz_),
      std::bind(&BatteryStateSim::update, this));

    RCLCPP_INFO(get_logger(), "BatteryStateSim started");
  }

private:
  // =========================
  // 回调：cmd_vel
  // =========================
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_linear_vel_  = std::hypot(msg->linear.x, msg->linear.y);
    last_angular_vel_ = std::abs(msg->angular.z);
  }

  // =========================
  // 定时更新
  // =========================
  void update()
  {
    // ===== 已没电：直接发布“没电状态” =====
    if (battery_empty_) {
      publishEmptyBattery();
      return;
    }

    double total_current = 0.0;

    if (discharge_enabled_) {
      // ===== 运动耗电 =====
      double motion_current =
        kv_ * last_linear_vel_ +
        kw_ * last_angular_vel_;

      // ===== 总电流 =====
      total_current =
        idle_current_ +
        camera_current_ +
        lidar_current_ +
        compute_current_gain_ * cpu_usage_ +
        communication_current_gain_ *
          (1.0 - bandwidth_mbps_ / std::max(1e-6, max_bandwidth_mbps_)) +
        motion_current;

      // ===== Ah 积分（线性放电）=====
      double dt_hours = (1.0 / update_rate_hz_) / 3600.0;
      current_ah_ -= total_current * dt_hours;

      // ===== 下限裁剪 + 锁存 =====
      if (current_ah_ <= design_capacity_ah_ * failure_charge_ratio_) {
        current_ah_ = design_capacity_ah_ * failure_charge_ratio_;
        battery_empty_ = true;
        publishEmptyBattery();
        return;
      }
    }

    // ===== 正常 BatteryState =====
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.voltage = 24.0;
    msg.current = total_current;
    msg.charge = current_ah_;
    msg.capacity = design_capacity_ah_;
    msg.design_capacity = design_capacity_ah_;
    msg.percentage = current_ah_ / design_capacity_ah_;

    msg.power_supply_status = discharge_enabled_
      ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING
      : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;

    msg.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;

    msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    msg.present = true;

    battery_pub_->publish(msg);
  }

  // =========================
  // 没电发布函数（核心）
  // =========================
  void publishEmptyBattery()
  {
    sensor_msgs::msg::BatteryState msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.voltage = 0.0;
    msg.current = 0.0;
    msg.charge = current_ah_;
    msg.capacity = design_capacity_ah_;
    msg.design_capacity = design_capacity_ah_;
    msg.percentage = failure_charge_ratio_;

    msg.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;

    msg.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;

    msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    msg.present = false;

    battery_pub_->publish(msg);
  }

  void resetBatteryCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    current_ah_ = design_capacity_ah_ * reset_charge_ratio_;
    battery_empty_ = false;
    last_linear_vel_ = 0.0;
    last_angular_vel_ = 0.0;

    response->success = true;
    response->message =
      "battery reset to " + std::to_string(reset_charge_ratio_ * 100.0) + "%";

    RCLCPP_INFO(
      get_logger(),
      "Battery reset: charge=%.2fAh (%.0f%%)",
      current_ah_,
      reset_charge_ratio_ * 100.0);

    update();
  }

  void setDischargeEnabledCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    discharge_enabled_ = request->data;
    if (!discharge_enabled_) {
      last_linear_vel_ = 0.0;
      last_angular_vel_ = 0.0;
    }

    response->success = true;
    response->message = discharge_enabled_ ? "battery discharge enabled" : "battery discharge paused";

    RCLCPP_INFO(
      get_logger(),
      "Battery discharge %s",
      discharge_enabled_ ? "enabled" : "paused");

    update();
  }

  // ===== ROS =====
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cpu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr bandwidth_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_battery_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_discharge_enabled_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ===== Parameters =====
  std::string frame_id_;
  double update_rate_hz_;
  double design_capacity_ah_;
  double initial_charge_ratio_;
  double reset_charge_ratio_;
  double failure_charge_ratio_;

  double camera_current_;
  double lidar_current_;
  double compute_current_gain_;
  double communication_current_gain_;
  double max_bandwidth_mbps_;
  double idle_current_;

  double kv_;
  double kw_;

  // ===== State =====
  double current_ah_;
  double last_linear_vel_{0.0};
  double last_angular_vel_{0.0};
  double cpu_usage_{0.25};
  double bandwidth_mbps_{10.0};
  bool discharge_enabled_{true};
  bool battery_empty_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryStateSim>());
  rclcpp::shutdown();
  return 0;
}
