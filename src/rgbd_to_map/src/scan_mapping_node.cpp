#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

class ScanToGridMapper : public rclcpp::Node
{
public:
  ScanToGridMapper() : Node("scan_to_grid_mapper")
  {
    this->declare_parameter<std::string>("scan_topic", "/scan1");
    this->declare_parameter<std::string>("grid_topic", "/local_grid");
    this->declare_parameter<std::string>("robot_frame", "base_link");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<double>("grid_resolution", 0.05);
    this->declare_parameter<int>("grid_width_min", 100);
    this->declare_parameter<int>("grid_height_min", 100);
    this->declare_parameter<int>("grid_width_max", 400);
    this->declare_parameter<int>("grid_height_max", 400);
    this->declare_parameter<double>("safety_margin", 1.0);
    this->declare_parameter<double>("scale_factor", 1.2);
    this->declare_parameter<double>("raytrace_step", 0.01);
    this->declare_parameter<bool>("dynamic_size", true);
    this->declare_parameter<double>("max_range_threshold", 10.0);
    
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    grid_topic_ = this->get_parameter("grid_topic").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    grid_resolution_ = this->get_parameter("grid_resolution").as_double();
    grid_width_min_ = this->get_parameter("grid_width_min").as_int();
    grid_height_min_ = this->get_parameter("grid_height_min").as_int();
    grid_width_max_ = this->get_parameter("grid_width_max").as_int();
    grid_height_max_ = this->get_parameter("grid_height_max").as_int();
    safety_margin_ = this->get_parameter("safety_margin").as_double();
    scale_factor_ = this->get_parameter("scale_factor").as_double();
    raytrace_step_ = this->get_parameter("raytrace_step").as_double();
    dynamic_size_ = this->get_parameter("dynamic_size").as_bool();
    max_range_threshold_ = this->get_parameter("max_range_threshold").as_double();
    
    grid_width_ = grid_width_min_;
    grid_height_ = grid_height_min_;
    
    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&ScanToGridMapper::scanCallback, this, std::placeholders::_1));
    
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    initializeGrid();
  }

private:
  void initializeGrid()
  {
    grid_data_.clear();
    grid_data_.resize(grid_width_ * grid_height_, -1);
    
    grid_origin_x_ = grid_width_ / 2.0 * grid_resolution_;
    grid_origin_y_ = grid_height_ / 2.0 * grid_resolution_;
    
    grid_msg_.header.frame_id = map_frame_;
    grid_msg_.info.resolution = grid_resolution_;
    grid_msg_.info.width = grid_width_;
    grid_msg_.info.height = grid_height_;
  }
  
  void resizeGridIfNeeded(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    if (!dynamic_size_) return;
    
    double max_valid_range = 0;
    int valid_count = 0;
    
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      float range = scan_msg->ranges[i];
      
      if (std::isfinite(range) && 
          range >= scan_msg->range_min && 
          range <= scan_msg->range_max &&
          range <= max_range_threshold_) {
        
        max_valid_range = std::max(max_valid_range, static_cast<double>(range));
        valid_count++;
      }
    }
    
    if (valid_count == 0) {
      max_valid_range = grid_width_min_ * grid_resolution_ / 2.0;
    }
    
    double required_radius = max_valid_range * scale_factor_ + safety_margin_;
    int required_width = static_cast<int>(2 * required_radius / grid_resolution_);
    int required_height = required_width;
    
    required_width = std::max(grid_width_min_, std::min(grid_width_max_, required_width));
    required_height = std::max(grid_height_min_, std::min(grid_height_max_, required_height));
    
    if (std::abs(required_width - grid_width_) > 10 || 
        std::abs(required_height - grid_height_) > 10) {
      
      grid_width_ = required_width;
      grid_height_ = required_height;
      
      initializeGrid();
    }
  }
  
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    resizeGridIfNeeded(scan_msg);
    
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        map_frame_, robot_frame_,
        scan_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &ex) {
      return;
    }
    
    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;
    double robot_yaw = getYawFromQuaternion(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    
    std::fill(grid_data_.begin(), grid_data_.end(), -1);
    
    grid_msg_.info.origin.position.x = robot_x - grid_origin_x_;
    grid_msg_.info.origin.position.y = robot_y - grid_origin_y_;
    grid_msg_.info.origin.position.z = 0.0;
    grid_msg_.info.origin.orientation.w = 1.0;
    
    processLaserScan(robot_x, robot_y, robot_yaw, scan_msg);
    
    grid_msg_.header.stamp = this->now();
    grid_msg_.data = grid_data_;
    grid_pub_->publish(grid_msg_);
  }
  
  void processLaserScan(double robot_x, double robot_y, double robot_yaw, 
                        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      float range = scan_msg->ranges[i];
      
      if (!std::isfinite(range) || 
          range < scan_msg->range_min || 
          range > scan_msg->range_max) {
        continue;
      }
      
      double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
      double beam_angle = robot_yaw + angle;
      
      double end_x = robot_x + range * cos(beam_angle);
      double end_y = robot_y + range * sin(beam_angle);
      
      raytrace(robot_x, robot_y, end_x, end_y);
      
      markCell(end_x, end_y, 100);
    }
  }
  
  void raytrace(double start_x, double start_y, double end_x, double end_y)
  {
    double dx = end_x - start_x;
    double dy = end_y - start_y;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (distance < grid_resolution_) return;
    
    dx /= distance;
    dy /= distance;
    
    double step = std::min(raytrace_step_, grid_resolution_ * 0.8);
    double current_x = start_x;
    double current_y = start_y;
    double traveled = 0.0;
    
    while (traveled < distance - grid_resolution_ * 0.5) {
      markCell(current_x, current_y, 0);
      
      current_x += dx * step;
      current_y += dy * step;
      traveled += step;
    }
  }
  
  void markCell(double world_x, double world_y, int value)
  {
    double grid_origin_x = grid_msg_.info.origin.position.x;
    double grid_origin_y = grid_msg_.info.origin.position.y;
    
    int grid_x = static_cast<int>((world_x - grid_origin_x) / grid_resolution_);
    int grid_y = static_cast<int>((world_y - grid_origin_y) / grid_resolution_);
    
    if (grid_x < 0 || grid_x >= grid_width_ || 
        grid_y < 0 || grid_y >= grid_height_) {
      return;
    }
    
    int index = grid_y * grid_width_ + grid_x;
    
    if (value == 100) {
      grid_data_[index] = value;
    } else if (grid_data_[index] == -1) {
      grid_data_[index] = value;
    }
  }
  
  double getYawFromQuaternion(double x, double y, double z, double w)
  {
    tf2::Quaternion q(x, y, z, w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  std::string scan_topic_;
  std::string grid_topic_;
  std::string robot_frame_;
  std::string map_frame_;
  double grid_resolution_;
  int grid_width_;
  int grid_height_;
  int grid_width_min_;
  int grid_height_min_;
  int grid_width_max_;
  int grid_height_max_;
  double safety_margin_;
  double scale_factor_;
  double raytrace_step_;
  bool dynamic_size_;
  double max_range_threshold_;
  
  std::vector<int8_t> grid_data_;
  nav_msgs::msg::OccupancyGrid grid_msg_;
  double grid_origin_x_;
  double grid_origin_y_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanToGridMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}