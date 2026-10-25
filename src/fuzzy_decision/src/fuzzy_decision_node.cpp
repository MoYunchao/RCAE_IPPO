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
#include <map>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>
#include <limits>

class FuzzyDecisionNode : public rclcpp::Node
{
public:
    FuzzyDecisionNode() : Node("fuzzy_decision_node"),
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

        // 30x30m 场景下建议先从 9.0 起步；越小越贪近，越大越愿意跑中远距离
        declare_parameter<double>("distance_norm_max", 15.0);

        // 新语义图范围：water=-0.5, corridor≈0.4, door=0.9
        declare_parameter<double>("semantic_score_min", -0.5);
        declare_parameter<double>("semantic_score_max", 1.0);

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

        RCLCPP_INFO(
            this->get_logger(),
            "模糊决策节点初始化，机器人ID: %d, weight_topic: %s",
            robot_id_,
            weight_topic_.c_str()
        );

        tf_buffer_->setUsingDedicatedThread(true);

        frontier_sub_ = this->create_subscription<frontier_msgs::msg::FrontierArrayExtended>(
            "/frontiers_info", 10,
            std::bind(&FuzzyDecisionNode::frontierCallback, this, std::placeholders::_1));

        weights_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            weight_topic_, 10,
            std::bind(&FuzzyDecisionNode::weightsCallback, this, std::placeholders::_1));

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
    double distance_norm_max_{9.0};
    double semantic_score_min_{-0.5};
    double semantic_score_max_{1.0};

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

    std::map<std::string, double> computeMembership(double value) {
        std::map<std::string, double> mu;
        mu["low"] = std::max(0.0, std::min(1.0, 1.0 - value));
        double medium = 1.0 - 2.0 * std::fabs(value - 0.5);
        mu["medium"] = std::max(0.0, std::min(1.0, medium));
        mu["high"] = std::max(0.0, std::min(1.0, value));
        return mu;
    }

    std::map<std::string, double> computeSemanticMembership(double s) {
        std::map<std::string, double> mu;
        s = std::clamp(s, 0.0, 1.0);

        if (s <= 0.20) {
            mu["low"] = 1.0;
        } else if (s < 0.45) {
            mu["low"] = (0.45 - s) / (0.45 - 0.20);
        } else {
            mu["low"] = 0.0;
        }

        if (s <= 0.25 || s >= 0.75) {
            mu["medium"] = 0.0;
        } else if (s < 0.50) {
            mu["medium"] = (s - 0.25) / (0.50 - 0.25);
        } else {
            mu["medium"] = (0.75 - s) / (0.75 - 0.50);
        }

        if (s <= 0.55) {
            mu["high"] = 0.0;
        } else if (s < 0.80) {
            mu["high"] = (s - 0.55) / (0.80 - 0.55);
        } else {
            mu["high"] = 1.0;
        }

        return mu;
    }

    std::map<std::string, double> computeDistanceMembership(double d) {
        std::map<std::string, double> mu;
        d = std::clamp(d, 0.0, 1.0);

        // 近距离：必须有明显优势，支持“本地先清扫再离开”
        // low: 左梯形 [0.00, 0.00, 0.28, 0.46]
        if (d <= 0.28) {
            mu["low"] = 1.0;
        } else if (d < 0.46) {
            mu["low"] = (0.46 - d) / (0.46 - 0.28);
        } else {
            mu["low"] = 0.0;
        }

        // 中距离：综合决策区
        // medium: 三角形 [0.32, 0.52, 0.70]
        if (d <= 0.32 || d >= 0.70) {
            mu["medium"] = 0.0;
        } else if (d <= 0.52) {
            mu["medium"] = (d - 0.32) / (0.52 - 0.32);
        } else {
            mu["medium"] = (0.70 - d) / (0.70 - 0.52);
        }

        // 远距离：强惩罚区
        // high: 右梯形 [0.60, 0.76, 1.00, 1.00]
        if (d <= 0.60) {
            mu["high"] = 0.0;
        } else if (d < 0.76) {
            mu["high"] = (d - 0.60) / (0.76 - 0.60);
        } else {
            mu["high"] = 1.0;
        }

        return mu;
    }

    std::map<std::string, double> computeExplorationGainMembership(double g) {
        std::map<std::string, double> mu;
        g = std::clamp(g, 0.0, 1.0);

        // 低 gain：面积偏小/收益偏低，要明显降权
        if (g <= 0.28) {
            mu["low"] = 1.0;
        } else if (g < 0.50) {
            mu["low"] = (0.50 - g) / (0.50 - 0.28);
        } else {
            mu["low"] = 0.0;
        }

        if (g <= 0.35 || g >= 0.78) {
            mu["medium"] = 0.0;
        } else if (g <= 0.56) {
            mu["medium"] = (g - 0.35) / (0.56 - 0.35);
        } else {
            mu["medium"] = (0.78 - g) / (0.78 - 0.56);
        }

        if (g <= 0.62) {
            mu["high"] = 0.0;
        } else if (g < 0.84) {
            mu["high"] = (g - 0.62) / (0.84 - 0.62);
        } else {
            mu["high"] = 1.0;
        }

        return mu;
    }

    std::map<std::string, double> fuzzyInference(
        const std::map<std::string, double>& sem_mu,
        const std::map<std::string, double>& dist_mu,
        const std::map<std::string, double>& gain_mu,
        const std::map<std::string, double>& risk_mu,
        const std::vector<double>& rl_weights)
    {
        std::map<std::string, double> output_mu = {
            {"极高", 0.0}, {"高", 0.0}, {"中", 0.0}, {"低", 0.0}
        };

        const double w_sem_raw  = (rl_weights.size() > 0) ? std::clamp(rl_weights[0], 0.0, 1.0) : 0.25;
        const double w_dist_raw = (rl_weights.size() > 1) ? std::clamp(rl_weights[1], 0.0, 1.0) : 0.25;
        const double w_gain_raw = (rl_weights.size() > 2) ? std::clamp(rl_weights[2], 0.0, 1.0) : 0.25;
        const double w_risk_raw = (rl_weights.size() > 3) ? std::clamp(rl_weights[3], 0.0, 1.0) : 0.25;

        // 近距离优先是主原则，因此给距离更高下限；risk 只做兜底
        const double w_sem  = std::max(0.15, w_sem_raw);
        const double w_dist = std::max(0.40, w_dist_raw);
        const double w_gain = std::max(0.22, w_gain_raw);
        const double w_risk = w_risk_raw;

        auto weightedMu = [](double mu, double w) {
            double scale = 0.5 + 2.0 * w;  // w=0.25 -> 1.0
            return std::clamp(mu * scale, 0.0, 1.0);
        };

        auto AND2 = [](double a, double b) {
            return std::min(a, b);
        };

        auto AND3 = [](double a, double b, double c) {
            return std::min(a, std::min(b, c));
        };

        auto S = [&](const std::string& label) { return weightedMu(sem_mu.at(label),  w_sem);  };
        auto D = [&](const std::string& label) { return weightedMu(dist_mu.at(label), w_dist); };
        auto G = [&](const std::string& label) { return weightedMu(gain_mu.at(label), w_gain); };
        auto R = [&](const std::string& label) { return weightedMu(risk_mu.at(label), w_risk); };

        auto applyRule = [&](const std::string& out_label, double strength) {
            output_mu[out_label] = std::max(output_mu[out_label], std::clamp(strength, 0.0, 1.0));
        };

        // =========================================================
        // 规则设计原则
        // 1) 近距离前沿优先，并且只有近距离才允许到“极高”
        // 2) 远距离强惩罚，尽量避免“本地还有前沿却先离开”
        // 3) 中距离进入语义+gain 综合决策区
        // 4) 小前沿（低 gain）显著降权，但近且高语义时允许保留机会
        // 5) risk_level 仅作兜底抑制
        // =========================================================

        // -------------------- A. 近距离：主导区 / 本地先清扫 --------------------
        // R1: 只要是近距离，就给一个高基础优先级，避免本地没清干净就离开
        applyRule("高", 0.92 * D("low"));

        // R2: 近 + 高收益 -> 极高
        applyRule("极高", AND2(D("low"), G("high")));

        // R3: 近 + 中收益 -> 高
        applyRule("高", AND2(D("low"), G("medium")));

        // R4: 近 + 低收益 -> 中
        applyRule("中", AND2(D("low"), G("low")));

        // R5: 近 + 高收益 + 中语义 -> 极高
        applyRule("极高", AND3(D("low"), G("high"), S("medium")));

        // R6: 近 + 高收益 + 高语义 -> 极高
        applyRule("极高", AND3(D("low"), G("high"), S("high")));

        // R7: 近 + 中收益 + 高语义 -> 高
        applyRule("高", AND3(D("low"), G("medium"), S("high")));

        // R8: 近 + 低收益 + 高语义 -> 中（保留门口/拐角类小入口）
        applyRule("中", AND3(D("low"), G("low"), S("high")));

        // -------------------- B. 中距离：综合决策区 --------------------
        // 中距离最多给到“高”，不允许压过近距离的“极高”
        // R9: 中 + 高收益 + 高语义 -> 高
        applyRule("高", AND3(D("medium"), G("high"), S("high")));

        // R10: 中 + 高收益 + 中语义 -> 高
        applyRule("高", AND3(D("medium"), G("high"), S("medium")));

        // R11: 中 + 高收益 + 低语义 -> 中
        applyRule("中", AND3(D("medium"), G("high"), S("low")));

        // R12: 中 + 中收益 + 高语义 -> 高
        applyRule("高", AND3(D("medium"), G("medium"), S("high")));

        // R13: 中 + 中收益 + 中语义 -> 中
        applyRule("中", AND3(D("medium"), G("medium"), S("medium")));

        // R14: 中 + 中收益 + 低语义 -> 中
        applyRule("中", AND3(D("medium"), G("medium"), S("low")));

        // R15: 中 + 低收益 + 高语义 -> 中
        applyRule("中", AND3(D("medium"), G("low"), S("high")));

        // R16: 中 + 低收益 + 中/低语义 -> 低
        applyRule("低", AND3(D("medium"), G("low"), S("medium")));
        applyRule("低", AND3(D("medium"), G("low"), S("low")));

        // -------------------- C. 远距离：强惩罚区 --------------------
        // 远距离最多给到“中”，尽量避免跳去远处再回头补本地
        // R17: 远 + 高收益 + 高语义 -> 中
        applyRule("中", AND3(D("high"), G("high"), S("high")));

        // R18: 远 + 高收益 + 中/低语义 -> 低
        applyRule("低", AND3(D("high"), G("high"), S("medium")));
        applyRule("低", AND3(D("high"), G("high"), S("low")));

        // R19: 远 + 中收益 -> 低
        applyRule("低", AND2(D("high"), G("medium")));

        // R20: 远 + 低收益 -> 低
        applyRule("低", AND2(D("high"), G("low")));

        // R21: 远距离本身就给基础低优先级
        applyRule("低", 0.88 * D("high"));

        // -------------------- D. 小前沿全局降权 --------------------
        // 默认压小前沿，减少为了很小收益专门回头补
        applyRule("低", G("low"));

        // 但近且高语义的小前沿允许保一个中等级，避免错杀入口型前沿
        applyRule("中", AND3(D("low"), G("low"), S("high")));

        // 对低 gain 做全局抑制：极高/高都明显压低
        const double low_gain = G("low");
        output_mu["极高"] *= (1.0 - 0.90 * low_gain);
        output_mu["高"]   *= (1.0 - 0.60 * low_gain);

        // -------------------- E. 风险兜底 --------------------
        const double risk_high = R("high");
        const double risk_med  = R("medium");

        output_mu["极高"] *= (1.0 - 0.85 * risk_high);
        output_mu["高"]   *= (1.0 - 0.60 * risk_high);
        output_mu["中"]   *= (1.0 - 0.25 * risk_high);

        applyRule("低", risk_high);
        applyRule("中", 0.20 * risk_med);

        return output_mu;
    }

    double defuzzify(const std::map<std::string, double>& output_mu) {
        double numerator = 0.0, denominator = 0.0;
        numerator += output_mu.at("极高") * weight_extremely_high_;
        numerator += output_mu.at("高") * weight_high_;
        numerator += output_mu.at("中") * weight_medium_;
        numerator += output_mu.at("低") * weight_low_;
        denominator += output_mu.at("极高") + output_mu.at("高") + output_mu.at("中") + output_mu.at("低");
        return (denominator > 1e-6) ? numerator / denominator : 0.0;
    }

    void weightsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 4) {
            std::vector<double> weights(msg->data.begin(), msg->data.end());
            bool invalid = false;
            for (double& w : weights) {
                if (!std::isfinite(w)) {
                    invalid = true;
                    break;
                }
                w = std::clamp(w, 0.0, 1.0);
            }
            if (invalid) {
                RCLCPP_WARN(this->get_logger(), "RL权重存在非法值，将回退到中性权重");
                weights = {0.5, 0.5, 0.5, 0.5};
            }
            rl_weights_ = weights;
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

        for (const auto& frontier : msg->frontiers_extended) {
            // ---------- 1. 提取并归一化四个特征 ----------
            double semantic_score_raw = frontier.semantic_score;
            double semantic_score = normalizeSemanticScore(semantic_score_raw);

            double distance = 0.0;
            if (static_cast<size_t>(robot_id_) < frontier.distances_to_robots.size()) {
                distance = frontier.distances_to_robots[robot_id_];
                distance = std::min(distance / std::max(distance_norm_max_, 1e-6), 1.0);
            } else {
                RCLCPP_WARN(this->get_logger(), "机器人ID %d 超出范围，跳过前沿点 %d", robot_id_, frontier.id);
                continue;
            }

            double exploration_gain = std::clamp(static_cast<double>(frontier.exploration_gain), 0.0, 1.0);
            double risk_level = std::clamp(static_cast<double>(frontier.risk_level), 0.0, 1.0);

            // ---------- 2. 坐标变换 ----------
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

            // ---------- 3. 局部地图可用性：仅作为轻度兜底 ----------
            bool usable_in_local_map = true;
            if (transform_success && has_local_map_) {
                usable_in_local_map = isFrontierUsableInLocalMap(local_point);
            }

            // ---------- 4. 计算隶属度 ----------
            auto sem_mu = computeSemanticMembership(semantic_score);
            auto dist_mu = computeDistanceMembership(distance);
            auto gain_mu = computeExplorationGainMembership(exploration_gain);
            auto risk_mu = computeMembership(risk_level);

            // ---------- 5. 模糊推理 ----------
            auto output_mu = fuzzyInference(sem_mu, dist_mu, gain_mu, risk_mu, rl_weights_);

            // 局部地图明显不可用时做轻微保守处理，但不一票否决
            if (!usable_in_local_map) {
                output_mu["极高"] *= 0.60;
                output_mu["高"]   *= 0.80;
                output_mu["低"] = std::max(output_mu["低"], 0.25);
            }

            // ---------- 6. 去模糊化 ----------
            double total_score = defuzzify(output_mu);

            // ---------- 7. 输出 ----------
            fuzzy_msgs::msg::FuzzyPriority priority_msg;
            priority_msg.frontier_id = frontier.id;
            priority_msg.total_score = total_score;
            priority_msg.weight_extremely_high = output_mu.at("极高");
            priority_msg.weight_high = output_mu.at("高");
            priority_msg.weight_medium = output_mu.at("中");
            priority_msg.weight_low = output_mu.at("低");
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
    auto node = std::make_shared<FuzzyDecisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
