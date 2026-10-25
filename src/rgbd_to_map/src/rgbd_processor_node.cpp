#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <vector>
#include <cmath>

class DepthImageToPointCloud : public rclcpp::Node
{
public:
    DepthImageToPointCloud() : Node("depth_image_to_pointcloud")
    {
        // 参数声明 - 使用你确认的RealSense R200参数
        this->declare_parameter<double>("depth_scale", 1.0);  // 32FC1浮点图，比例=1.0
        this->declare_parameter<std::string>("target_frame", "base_link");
        this->declare_parameter<std::string>("camera_frame", "oakd_rgb_camera_frame");
        
        // RealSense R200实际内参（从camera_info获取）
        this->declare_parameter<double>("fx", 277.0);
        this->declare_parameter<double>("fy", 277.0);
        this->declare_parameter<double>("cx", 160.0);
        this->declare_parameter<double>("cy", 120.0);

        // 获取参数
        depth_scale_ = this->get_parameter("depth_scale").as_double();
        target_frame_ = this->get_parameter("target_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        intrinsics_.fx = this->get_parameter("fx").as_double();
        intrinsics_.fy = this->get_parameter("fy").as_double();
        intrinsics_.cx = this->get_parameter("cx").as_double();
        intrinsics_.cy = this->get_parameter("cy").as_double();

        // 创建发布器
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "tum_pointcloud", 10);

        // 创建订阅器
        depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/oakd/rgb/preview/depth", 10,
            std::bind(&DepthImageToPointCloud::depthImageCallback, this, std::placeholders::_1));

        // TF2监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(this->get_logger(), "Depth Image to PointCloud started (Python logic)");
        RCLCPP_INFO(this->get_logger(), "Camera: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                   intrinsics_.fx, intrinsics_.fy, intrinsics_.cx, intrinsics_.cy);
        RCLCPP_INFO(this->get_logger(), "Depth scale: %.2f, Frames: %s -> %s",
                   depth_scale_, camera_frame_.c_str(), target_frame_.c_str());
    }

private:
    struct Intrinsics {
        double fx, fy, cx, cy;
    };

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 获取深度图像的时间戳
            rclcpp::Time image_timestamp = msg->header.stamp;
            std::string source_frame = msg->header.frame_id.empty() ? camera_frame_ : msg->header.frame_id;
            
            // 关键修正：处理32FC1浮点深度图
            cv_bridge::CvImagePtr cv_ptr;
            try {
                // 深度图是32FC1浮点格式
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            
            cv::Mat depth_image = cv_ptr->image;
            
            // 使用Python代码的逻辑生成点云
            auto cloud = depthImageToPointCloudPythonStyle(depth_image);
            
            if (cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "Generated empty point cloud");
                return;
            }

            RCLCPP_DEBUG(this->get_logger(), "Generated %zu points from %dx%d image",
                        cloud->size(), depth_image.cols, depth_image.rows);

            // 创建点云消息
            auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            pcl::toROSMsg(*cloud, *cloud_msg);
            cloud_msg->header.stamp = image_timestamp;
            cloud_msg->header.frame_id = source_frame;
            
            // 坐标系转换
            if (target_frame_ != source_frame) {
                try {
                    // 总是尝试使用最新的TF变换
                    auto transform = tf_buffer_->lookupTransform(
                        target_frame_,
                        source_frame,
                        rclcpp::Time(0),  
                        rclcpp::Duration::from_seconds(0.1)
                    );
                    
                    auto transformed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                    tf2::doTransform(*cloud_msg, *transformed_msg, transform);
                    transformed_msg->header.frame_id = target_frame_;
                    transformed_msg->header.stamp = image_timestamp;  
                    
                    pointcloud_pub_->publish(*transformed_msg);
                    
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
                    pointcloud_pub_->publish(*cloud_msg);
                }
            } else {
                pointcloud_pub_->publish(*cloud_msg);
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error processing depth image: %s", e.what());
        }
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr depthImageToPointCloudPythonStyle(const cv::Mat& depth_image)
    {
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        
        int height = depth_image.rows;
        int width = depth_image.cols;
        
        // 预分配内存（估计最多80%的像素有效）
        cloud->points.reserve(height * width * 0.8);
        
        // Python逻辑：遍历所有像素，过滤有效深度
        for (int v = 0; v < height; v++) {
            for (int u = 0; u < width; u++) {
                // 关键：读取32位浮点数
                float depth_value = depth_image.at<float>(v, u);
                
                // 跳过无效深度（Python代码中 depth_image > 0 的逻辑）
                if (depth_value <= 0.0 || std::isnan(depth_value)) {
                    continue;
                }
                
                // Python逻辑：z = depth_image[valid_depth] / depth_scale
                // 对于32FC1，depth_scale=1.0，所以z直接等于depth_value
                float z = depth_value / depth_scale_;
                
                // 可选：跳过太远或太近的点
                if (z < 0.1 || z > 10.0) {
                    continue;
                }
                
                // Python逻辑：x = (u - cx) * z / fx
                float x = (u - intrinsics_.cx) * z / intrinsics_.fx;
                float y = (v - intrinsics_.cy) * z / intrinsics_.fy;
                
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;
                
                cloud->points.push_back(point);
            }
        }
        
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;  // 所有点都有效
        
        return cloud;
    }

    // 成员变量
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    double depth_scale_;
    std::string target_frame_;
    std::string camera_frame_;
    Intrinsics intrinsics_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthImageToPointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}