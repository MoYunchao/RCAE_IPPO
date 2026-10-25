#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

class BatteryManager : public rclcpp::Node
{
public:
  BatteryManager()
  : Node("battery_manager")
  {
    // ===== Parameters =====
    declare_parameter("auto_recover", false);

    // 只保留 nav2_nodes 参数，移除 slam_node 参数
    declare_parameter(
      "nav2_nodes",
      std::vector<std::string>{
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother"
      });

    get_parameter("auto_recover", auto_recover_);
    get_parameter("nav2_nodes", nav2_nodes_);

    // ===== Subscriber =====
    battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
      "battery_state",   // 自动适配 namespace
      rclcpp::QoS(10),
      std::bind(&BatteryManager::batteryCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    failed_pub_ = create_publisher<std_msgs::msg::Bool>(
      "failed", rclcpp::QoS(1).reliable().transient_local());
    publishFailed(false);

    RCLCPP_INFO(get_logger(), "Battery manager started");
    RCLCPP_INFO(get_logger(), "Managing Nav2 nodes only, SLAM node will not be controlled");
  }

private:
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
  {
    // === 没电 ===
    if (!msg->present && system_active_) {
      RCLCPP_WARN(get_logger(), "Battery depleted. Robot permanently failed.");
      deactivateNav2();
      system_active_ = false;
      publishFailed(true);
      return;
    }

    // === 来电恢复 ===
    if (msg->present && !system_active_ && auto_recover_) {
      RCLCPP_INFO(get_logger(), "Battery restored. Reactivating Nav2 system...");
      activateNav2();
      system_active_ = true;
      return;
    }
  }

  void deactivateNav2()
  {
    for (const auto & node : nav2_nodes_) {
      changeState(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
    auto zero_twist = std::make_unique<geometry_msgs::msg::Twist>();
    zero_twist->linear.x = 0.0;
    zero_twist->linear.y = 0.0;
    zero_twist->linear.z = 0.0;
    zero_twist->angular.x = 0.0;
    zero_twist->angular.y = 0.0;
    zero_twist->angular.z = 0.0;
    
    cmd_vel_pub_->publish(std::move(zero_twist));
    RCLCPP_INFO(get_logger(), "Published zero velocity command to cmd_vel");
  }



  void activateNav2()
  {
    for (const auto & node : nav2_nodes_) {
      changeState(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }
  }

  void publishFailed(bool failed)
  {
    std_msgs::msg::Bool msg;
    msg.data = failed;
    failed_pub_->publish(msg);
  }

  void changeState(const std::string & node_name, uint8_t transition)
  {
    std::string service_name = node_name + "/change_state";

    auto client = create_client<lifecycle_msgs::srv::ChangeState>(service_name);

    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(
        get_logger(),
        "Lifecycle service not available: %s",
        service_name.c_str());
      return;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    client->async_send_request(request);
  }

  // ===== ROS =====
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr failed_pub_;

  // ===== Parameters =====
  bool auto_recover_;
  std::vector<std::string> nav2_nodes_;

  // ===== State =====
  bool system_active_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryManager>());
  rclcpp::shutdown();
  return 0;
}
