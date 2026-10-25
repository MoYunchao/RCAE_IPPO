#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <cmath>

class PointCloudToLaserScan : public rclcpp::Node
{
public:
  PointCloudToLaserScan()
  : Node("pointcloud_to_laserscan")
  {
    this->declare_parameter<std::string>("input_topic", "/oakd/rgb/preview/depth/points");
    this->declare_parameter<std::string>("output_topic", "/scan1");
    this->declare_parameter<std::string>("target_frame", "");

    this->declare_parameter<double>("min_height", 0.0);
    this->declare_parameter<double>("max_height", 0.5);

    this->declare_parameter<double>("angle_min", -M_PI / 2.0);
    this->declare_parameter<double>("angle_max",  M_PI / 2.0);
    this->declare_parameter<double>("angle_increment", M_PI / 1080.0);

    this->declare_parameter<double>("scan_time", 0.0333333);
    this->declare_parameter<double>("range_min", 0.1);
    this->declare_parameter<double>("range_max", 30.0);

    this->declare_parameter<bool>("use_inf", true);
    this->declare_parameter<double>("inf_epsilon", 1e-6);

    /* ===== 体素滤波参数 ===== */
    this->declare_parameter<double>("voxel_leaf_size", 0.05);

    input_topic_   = this->get_parameter("input_topic").as_string();
    output_topic_  = this->get_parameter("output_topic").as_string();
    target_frame_  = this->get_parameter("target_frame").as_string();

    min_height_    = this->get_parameter("min_height").as_double();
    max_height_    = this->get_parameter("max_height").as_double();

    angle_min_     = this->get_parameter("angle_min").as_double();
    angle_max_     = this->get_parameter("angle_max").as_double();
    angle_increment_ = this->get_parameter("angle_increment").as_double();

    scan_time_     = this->get_parameter("scan_time").as_double();
    range_min_     = this->get_parameter("range_min").as_double();
    range_max_     = this->get_parameter("range_max").as_double();

    use_inf_       = this->get_parameter("use_inf").as_bool();
    inf_epsilon_   = this->get_parameter("inf_epsilon").as_double();

    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic_, rclcpp::QoS(10));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&PointCloudToLaserScan::cloudCallback, this, std::placeholders::_1));

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(),
      "PointCloudToLaserScan with VoxelGrid enabled (leaf_size=%.3f m)",
      voxel_leaf_size_);
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    /* ================================
     * 1. PointCloud2 → PCL
     * ================================ */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    /* ================================
     * 2. 体素滤波（在 TF 之前）
     * ================================ */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(pcl_cloud);
    voxel.setLeafSize(
      voxel_leaf_size_,
      voxel_leaf_size_,
      voxel_leaf_size_);
    voxel.filter(*pcl_filtered);

    sensor_msgs::msg::PointCloud2 cloud_ds;
    pcl::toROSMsg(*pcl_filtered, cloud_ds);
    cloud_ds.header = cloud_msg->header;

    /* ================================
     * 3. LaserScan 初始化
     * ================================ */
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header = cloud_ds.header;

    if (!target_frame_.empty() &&
        target_frame_ != cloud_ds.header.frame_id)
      scan_msg.header.frame_id = target_frame_;
    else
      scan_msg.header.frame_id = cloud_ds.header.frame_id;

    scan_msg.angle_min = angle_min_;
    scan_msg.angle_max = angle_max_;
    scan_msg.angle_increment = angle_increment_;
    scan_msg.scan_time = scan_time_;
    scan_msg.range_min = range_min_;
    scan_msg.range_max = range_max_;

    const int ranges_size =
      static_cast<int>(std::round(
        (angle_max_ - angle_min_) / angle_increment_)) + 1;

    scan_msg.ranges.assign(
      ranges_size,
      use_inf_
        ? std::numeric_limits<float>::infinity()
        : float(range_max_ + inf_epsilon_));

    /* ================================
     * 4. TF 变换（只对降采样后的点云）
     * ================================ */
    sensor_msgs::msg::PointCloud2 cloud_in = cloud_ds;

    if (scan_msg.header.frame_id != cloud_ds.header.frame_id)
    {
      try {
        auto tf = tf_buffer_->lookupTransform(
          scan_msg.header.frame_id,
          cloud_ds.header.frame_id,
          cloud_ds.header.stamp,
          rclcpp::Duration::from_seconds(0.1));

        sensor_msgs::msg::PointCloud2 tmp;
        tf2::doTransform(cloud_in, tmp, tf);
        cloud_in = tmp;
      }
      catch (const tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "TF transform failed: %s", ex.what());
        scan_pub_->publish(scan_msg);
        return;
      }
    }

    /* ================================
     * 5. 投影到 LaserScan
     * ================================ */
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_in, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      const float X = *iter_x;
      const float Y = *iter_y;
      const float Z = *iter_z;

      if (!std::isfinite(X) || !std::isfinite(Y) || !std::isfinite(Z))
        continue;

      if (Z < min_height_ || Z > max_height_)
        continue;

      const float range = std::hypot(X, Y);
      if (range < range_min_ || range > range_max_)
        continue;

      const float angle = std::atan2(Y, X);
      if (angle < angle_min_ || angle > angle_max_)
        continue;

      const int index =
        static_cast<int>((angle - angle_min_) / angle_increment_);

      if (index >= 0 && index < ranges_size)
      {
        if (range < scan_msg.ranges[index])
          scan_msg.ranges[index] = range;
      }
    }

    scan_pub_->publish(scan_msg);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;

  double min_height_;
  double max_height_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  double range_min_;
  double range_max_;
  bool   use_inf_;
  double inf_epsilon_;

  double voxel_leaf_size_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToLaserScan>());
  rclcpp::shutdown();
  return 0;
}