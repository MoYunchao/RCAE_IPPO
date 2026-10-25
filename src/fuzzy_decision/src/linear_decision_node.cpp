#include "rclcpp/rclcpp.hpp"
#include "frontier_msgs/msg/frontier_array_extended.hpp"
#include "fuzzy_msgs/msg/fuzzy_priority.hpp"
#include "fuzzy_msgs/msg/fuzzy_priority_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"

#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>

class LinearDecisionNode : public rclcpp::Node
{
public:
    LinearDecisionNode() : Node("linear_decision_node"),
        tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        declare_parameter<double>("weight_extremely_high", 10.0);
        declare_parameter<double>("weight_high", 8.0);
        declare_parameter<double>("weight_medium", 5.0);
        declare_parameter<double>("weight_low", 2.0);

        declare_parameter<int>("robot_id", 0);

        declare_parameter<double>("tf_timeout", 1.0);
        declare_parameter<double>("tf_cache_time", 10.0);

        declare_parameter<std::string>("local_map_topic","/robot_0/map");
        declare_parameter<std::string>("local_map_frame","robot_0/map");
        declare_parameter<std::string>("weight_topic", "/rl_adjust_weights");
        declare_parameter<bool>("wait_for_rl_weights", true);

        declare_parameter<double>("distance_norm_max", 15.0);
        declare_parameter<double>("semantic_score_min", -0.5);
        declare_parameter<double>("semantic_score_max", 1.0);
        declare_parameter<double>("unusable_score_scale", 0.8);

        weight_extremely_high_ = get_parameter("weight_extremely_high").as_double();
        weight_high_ = get_parameter("weight_high").as_double();
        weight_medium_ = get_parameter("weight_medium").as_double();
        weight_low_ = get_parameter("weight_low").as_double();
        robot_id_ = get_parameter("robot_id").as_int();
        tf_timeout_ = get_parameter("tf_timeout").as_double();
        tf_cache_time_ = get_parameter("tf_cache_time").as_double();
        local_map_topic_ = get_parameter("local_map_topic").as_string();
        local_map_frame_ = get_parameter("local_map_frame").as_string();
        weight_topic_ = get_parameter("weight_topic").as_string();
        wait_for_rl_weights_ = get_parameter("wait_for_rl_weights").as_bool();
        distance_norm_max_ = get_parameter("distance_norm_max").as_double();
        semantic_score_min_ = get_parameter("semantic_score_min").as_double();
        semantic_score_max_ = get_parameter("semantic_score_max").as_double();
        unusable_score_scale_ = std::clamp(
            get_parameter("unusable_score_scale").as_double(), 0.0, 1.0);

        RCLCPP_INFO(
            this->get_logger(),
            "线性决策节点初始化，robot_id=%d, weight_topic=%s",
            robot_id_,
            weight_topic_.c_str());

        tf_buffer_->setUsingDedicatedThread(true);

        frontier_sub_ = this->create_subscription<frontier_msgs::msg::FrontierArrayExtended>(
            "/frontiers_info", 10,
            std::bind(&LinearDecisionNode::frontierCallback, this, std::placeholders::_1));

        weights_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            weight_topic_, 10,
            std::bind(&LinearDecisionNode::weightsCallback, this, std::placeholders::_1));

        rl_weights_ = {0.25, 0.25, 0.25, 0.25};
        rl_weights_received_ = false;

        local_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            local_map_topic_, 10,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                local_map_ = msg;
                has_local_map_ = true;
            }
        );

        priority_pub_ = this->create_publisher<fuzzy_msgs::msg::FuzzyPriorityArray>(
            "/frontier_priority_fuzzy", 10);
    }

private:
    rclcpp::Subscription<frontier_msgs::msg::FrontierArrayExtended>::SharedPtr frontier_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr weights_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;
    rclcpp::Publisher<fuzzy_msgs::msg::FuzzyPriorityArray>::SharedPtr priority_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double weight_extremely_high_;
    double weight_high_;
    double weight_medium_;
    double weight_low_;
    int robot_id_;
    double tf_timeout_;
    double tf_cache_time_;
    std::string local_map_topic_;
    std::string local_map_frame_;
    std::string weight_topic_;
    bool wait_for_rl_weights_{true};
    double distance_norm_max_{15.0};
    double semantic_score_min_{-0.5};
    double semantic_score_max_{1.0};
    double unusable_score_scale_{0.8};

    nav_msgs::msg::OccupancyGrid::SharedPtr local_map_;
    bool has_local_map_{false};

    std::vector<double> rl_weights_;
    bool rl_weights_received_;

    bool transformToLocal(
        const geometry_msgs::msg::Point& global_point,
        geometry_msgs::msg::Point& local_point,
        const rclcpp::Time& transform_time) {
        try {
            std::string target_frame = "robot_" + std::to_string(robot_id_) + "/map";
            std::string source_frame = "global_map";

            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform(
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

        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(),
                "TF变换异常 (robot_id=%d): %s", robot_id_, ex.what());
            return false;
        }
    }

    bool isFrontierUsableInLocalMap(const geometry_msgs::msg::Point& local_point) {
        if(!has_local_map_ || !local_map_){
            return false;
        }

        const auto& info = local_map_->info;
        const auto& data = local_map_->data;
        int cx = static_cast<int>((local_point.x - info.origin.position.x) / info.resolution);
        int cy = static_cast<int>((local_point.y - info.origin.position.y) / info.resolution);

        if(cx < 0 || cy < 0 || cx >= static_cast<int>(info.width) || cy >= static_cast<int>(info.height)){
            return false;
        }

        for (int dx = -3; dx <= 3; ++dx){
            for (int dy = -3; dy <= 3; ++dy){
                int nx = cx + dx;
                int ny = cy + dy;
                if(nx < 0 || ny < 0 || nx >= static_cast<int>(info.width) || ny >= static_cast<int>(info.height)){
                    continue;
                }

                int idx = ny * static_cast<int>(info.width) + nx;
                if(data[idx] == 0){
                    return true;
                }
            }
        }

        return false;
    }

    double normalizeSemanticScore(double raw_score) const {
        double denom = std::max(semantic_score_max_ - semantic_score_min_, 1e-6);
        double norm = (raw_score - semantic_score_min_) / denom;
        return std::clamp(norm, 0.0, 1.0);
    }

    std::vector<double> normalizeWeights(const std::vector<double>& raw_weights) const {
        std::vector<double> weights = raw_weights;
        if (weights.size() != 4) {
            return {0.25, 0.25, 0.25, 0.25};
        }

        bool invalid = false;
        for (double& w : weights) {
            if (!std::isfinite(w)) {
                invalid = true;
                break;
            }
            w = std::clamp(w, 0.0, 1.0);
        }

        if (invalid) {
            return {0.25, 0.25, 0.25, 0.25};
        }

        const double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
        if (sum <= 1e-9) {
            return {0.25, 0.25, 0.25, 0.25};
        }

        for (double& w : weights) {
            w /= sum;
        }
        return weights;
    }

    void weightsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 4) {
            std::vector<double> weights(msg->data.begin(), msg->data.end());
            rl_weights_ = normalizeWeights(weights);
            rl_weights_received_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "RL权重维度错误，期望4，实际%zu", msg->data.size());
        }
    }

    void frontierCallback(const frontier_msgs::msg::FrontierArrayExtended::SharedPtr msg) {
        if (wait_for_rl_weights_ && !rl_weights_received_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "尚未收到 RL 权重，当前先不发布 frontier_priority_fuzzy，等待 %s",
                weight_topic_.c_str()
            );
            return;
        }

        fuzzy_msgs::msg::FuzzyPriorityArray priority_array;
        priority_array.header = msg->header;

        const std::vector<double> active_weights = normalizeWeights(rl_weights_);

        for (const auto& frontier : msg->frontiers_extended) {
            double semantic_score_raw = frontier.semantic_score;
            double semantic_term = normalizeSemanticScore(semantic_score_raw);

            double distance_norm = 1.0;
            if (static_cast<size_t>(robot_id_) < frontier.distances_to_robots.size()) {
                const double raw_distance = std::max(0.0, frontier.distances_to_robots[robot_id_]);
                distance_norm = std::min(raw_distance / std::max(distance_norm_max_, 1e-6), 1.0);
            } else {
                RCLCPP_WARN(this->get_logger(), "机器人ID %d 超出范围，跳过前沿点 %d", robot_id_, frontier.id);
                continue;
            }

            const double distance_term = 1.0 - distance_norm;
            const double gain_term = std::clamp(static_cast<double>(frontier.exploration_gain), 0.0, 1.0);
            const double risk_term = 1.0 - std::clamp(static_cast<double>(frontier.risk_level), 0.0, 1.0);

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

            const double sem_component  = active_weights[0] * semantic_term;
            const double dist_component = active_weights[1] * distance_term;
            const double gain_component = active_weights[2] * gain_term;
            const double risk_component = active_weights[3] * risk_term;

            double total_score_norm =
                sem_component + dist_component + gain_component + risk_component;
            total_score_norm = std::clamp(total_score_norm, 0.0, 1.0);

            if (!usable_in_local_map) {
                total_score_norm *= unusable_score_scale_;
            }

            const double score_span = weight_extremely_high_ - weight_low_;
            const double total_score = weight_low_ + score_span * total_score_norm;

            fuzzy_msgs::msg::FuzzyPriority priority_msg;
            priority_msg.frontier_id = frontier.id;
            priority_msg.total_score = total_score;
            priority_msg.weight_extremely_high = sem_component;
            priority_msg.weight_high = dist_component;
            priority_msg.weight_medium = gain_component;
            priority_msg.weight_low = risk_component;
            priority_msg.global_position = global_point;
            priority_msg.local_position = local_point;
            priority_msg.transform_successful = transform_success;

            priority_array.priorities.push_back(priority_msg);
        }

        priority_pub_->publish(priority_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LinearDecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
