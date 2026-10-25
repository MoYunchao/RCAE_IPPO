#include "rclcpp/rclcpp.hpp"
#include "frontier_msgs/msg/frontier_array_extended.hpp"
#include "fuzzy_msgs/msg/fuzzy_priority.hpp"
#include "fuzzy_msgs/msg/fuzzy_priority_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

class GreedyDecisionNode : public rclcpp::Node
{
public:
    GreedyDecisionNode()
    : Node("greedy_decision_node"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        declare_parameter<double>("weight_extremely_high", 10.0);
        declare_parameter<double>("weight_low", 2.0);
        declare_parameter<int>("robot_id", 0);
        declare_parameter<double>("tf_timeout", 1.0);
        declare_parameter<std::string>("local_map_topic", "/robot_0/map");
        declare_parameter<std::string>("local_map_frame", "robot_0/map");
        declare_parameter<double>("distance_norm_max", 15.0);
        declare_parameter<double>("unusable_score_scale", 0.8);

        weight_extremely_high_ = get_parameter("weight_extremely_high").as_double();
        weight_low_ = get_parameter("weight_low").as_double();
        robot_id_ = get_parameter("robot_id").as_int();
        tf_timeout_ = get_parameter("tf_timeout").as_double();
        local_map_topic_ = get_parameter("local_map_topic").as_string();
        local_map_frame_ = get_parameter("local_map_frame").as_string();
        distance_norm_max_ = get_parameter("distance_norm_max").as_double();
        unusable_score_scale_ =
            std::clamp(get_parameter("unusable_score_scale").as_double(), 0.0, 1.0);

        RCLCPP_INFO(
            this->get_logger(),
            "纯距离贪心决策节点初始化，robot_id=%d, local_map_topic=%s",
            robot_id_,
            local_map_topic_.c_str());

        tf_buffer_->setUsingDedicatedThread(true);

        frontier_sub_ = this->create_subscription<frontier_msgs::msg::FrontierArrayExtended>(
            "/frontiers_info", 10,
            std::bind(&GreedyDecisionNode::frontierCallback, this, std::placeholders::_1));

        local_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            local_map_topic_, 10,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                local_map_ = msg;
                has_local_map_ = true;
            });

        priority_pub_ = this->create_publisher<fuzzy_msgs::msg::FuzzyPriorityArray>(
            "/frontier_priority_fuzzy", 10);
    }

private:
    rclcpp::Subscription<frontier_msgs::msg::FrontierArrayExtended>::SharedPtr frontier_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;
    rclcpp::Publisher<fuzzy_msgs::msg::FuzzyPriorityArray>::SharedPtr priority_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double weight_extremely_high_{10.0};
    double weight_low_{2.0};
    int robot_id_{0};
    double tf_timeout_{1.0};
    std::string local_map_topic_;
    std::string local_map_frame_;
    double distance_norm_max_{15.0};
    double unusable_score_scale_{0.8};

    nav_msgs::msg::OccupancyGrid::SharedPtr local_map_;
    bool has_local_map_{false};

    bool transformToLocal(
        const geometry_msgs::msg::Point & global_point,
        geometry_msgs::msg::Point & local_point,
        const rclcpp::Time & transform_time)
    {
        try {
            const std::string target_frame = "robot_" + std::to_string(robot_id_) + "/map";
            const std::string source_frame = "global_map";

            const auto transform = tf_buffer_->lookupTransform(
                target_frame,
                source_frame,
                transform_time,
                rclcpp::Duration::from_seconds(tf_timeout_));

            geometry_msgs::msg::PointStamped global_stamped, local_stamped;
            global_stamped.header.stamp = transform_time;
            global_stamped.header.frame_id = source_frame;
            global_stamped.point = global_point;

            tf2::doTransform(global_stamped, local_stamped, transform);
            local_point = local_stamped.point;
            return true;
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "TF变换异常 (robot_id=%d): %s", robot_id_, ex.what());
            return false;
        }
    }

    bool isFrontierUsableInLocalMap(const geometry_msgs::msg::Point & local_point) const
    {
        if (!has_local_map_ || !local_map_) {
            return false;
        }

        const auto & info = local_map_->info;
        const auto & data = local_map_->data;
        const int cx = static_cast<int>((local_point.x - info.origin.position.x) / info.resolution);
        const int cy = static_cast<int>((local_point.y - info.origin.position.y) / info.resolution);

        if (cx < 0 || cy < 0 || cx >= static_cast<int>(info.width) ||
            cy >= static_cast<int>(info.height))
        {
            return false;
        }

        for (int dx = -3; dx <= 3; ++dx) {
            for (int dy = -3; dy <= 3; ++dy) {
                const int nx = cx + dx;
                const int ny = cy + dy;
                if (nx < 0 || ny < 0 || nx >= static_cast<int>(info.width) ||
                    ny >= static_cast<int>(info.height))
                {
                    continue;
                }

                const int idx = ny * static_cast<int>(info.width) + nx;
                if (data[idx] == 0) {
                    return true;
                }
            }
        }

        return false;
    }

    void frontierCallback(const frontier_msgs::msg::FrontierArrayExtended::SharedPtr msg)
    {
        fuzzy_msgs::msg::FuzzyPriorityArray priority_array;
        priority_array.header = msg->header;

        for (const auto & frontier : msg->frontiers_extended) {
            if (static_cast<size_t>(robot_id_) >= frontier.distances_to_robots.size()) {
                RCLCPP_WARN(this->get_logger(), "机器人ID %d 超出范围，跳过前沿点 %d", robot_id_, frontier.id);
                continue;
            }

            const double raw_distance =
                std::max(0.0, static_cast<double>(frontier.distances_to_robots[robot_id_]));
            const double distance_norm =
                std::min(raw_distance / std::max(distance_norm_max_, 1e-6), 1.0);
            const double distance_term = 1.0 - distance_norm;

            geometry_msgs::msg::Point global_point, local_point;
            global_point.x = frontier.position[0];
            global_point.y = frontier.position[1];
            global_point.z = 0.0;

            bool transform_success = false;
            if (tf_buffer_) {
                transform_success = transformToLocal(global_point, local_point, msg->header.stamp);
                if (!transform_success) {
                    local_point = global_point;
                }
            } else {
                local_point = global_point;
            }

            bool usable_in_local_map = true;
            if (transform_success && has_local_map_) {
                usable_in_local_map = isFrontierUsableInLocalMap(local_point);
            }

            double total_score_norm = std::clamp(distance_term, 0.0, 1.0);
            if (!usable_in_local_map) {
                total_score_norm *= unusable_score_scale_;
            }

            const double score_span = weight_extremely_high_ - weight_low_;
            const double total_score = weight_low_ + score_span * total_score_norm;

            fuzzy_msgs::msg::FuzzyPriority priority_msg;
            priority_msg.frontier_id = frontier.id;
            priority_msg.total_score = total_score;
            priority_msg.weight_extremely_high = 0.0;
            priority_msg.weight_high = distance_term;
            priority_msg.weight_medium = 0.0;
            priority_msg.weight_low = 0.0;
            priority_msg.global_position = global_point;
            priority_msg.local_position = local_point;
            priority_msg.transform_successful = transform_success;
            priority_array.priorities.push_back(priority_msg);
        }

        priority_pub_->publish(priority_array);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GreedyDecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
