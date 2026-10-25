#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>
#include <memory>

class LaserScanFrameTransformer : public rclcpp::Node
{
public:
  LaserScanFrameTransformer() : Node("laserscan_frame_transformer")
  {
    // 声明参数
    this->declare_parameter<std::string>("input_topic", "/scan");
    this->declare_parameter<std::string>("output_topic", "/scan_transformed");
    this->declare_parameter<std::string>("output_frame_id", "robot_0/depth_camera/link");
    this->declare_parameter<bool>("republish_header_stamp", true);

    // 获取参数值
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    output_frame_id_ = this->get_parameter("output_frame_id").as_string();
    republish_header_stamp_ = this->get_parameter("republish_header_stamp").as_bool();

    // 创建发布者和订阅者
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::QoS(10));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&LaserScanFrameTransformer::scanCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), 
                "LaserScan Frame Transformer initialized.\n"
                "  Input topic: %s\n"
                "  Output topic: %s\n"
                "  Output frame_id: %s",
                input_topic_.c_str(), output_topic_.c_str(), output_frame_id_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 创建新的消息副本
    auto transformed_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    
    // 修改 frame_id
    transformed_msg->header.frame_id = output_frame_id_;
    
    // 如果需要，更新时间戳为当前时间
    if (republish_header_stamp_) {
      transformed_msg->header.stamp = this->now();
    }
    
    // 发布转换后的消息
    scan_pub_->publish(*transformed_msg);
    
    // 可选：记录日志（调试用，生产环境建议关闭）
    static size_t count = 0;
    if (++count % 100 == 0) {
      RCLCPP_DEBUG(this->get_logger(), 
                   "Transformed scan #%zu: frame_id from '%s' to '%s'",
                   count, msg->header.frame_id.c_str(), output_frame_id_.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool republish_header_stamp_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserScanFrameTransformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}