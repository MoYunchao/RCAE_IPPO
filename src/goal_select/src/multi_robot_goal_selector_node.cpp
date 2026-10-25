#include "rclcpp/rclcpp.hpp"
#include "fuzzy_msgs/msg/fuzzy_priority_array.hpp"
#include "fuzzy_msgs/msg/fuzzy_priority.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <cmath>
#include <limits>
#include <unordered_map>

class MultiRobotGoalSelectorNode : public rclcpp::Node
{
public:
    MultiRobotGoalSelectorNode() : Node("multi_robot_goal_selector_node")
    {
        // 声明参数
        declare_parameter<double>("position_tolerance", 1.0);  // 位置容差，判断是否为同一点
        declare_parameter<double>("startup_warmup_sec", 8.0);  // 三台机器人都拿到非空前沿后，再额外等待一段时间
        declare_parameter<bool>("require_all_robots_ready", true);
        declare_parameter<double>("failed_frontier_cooldown_sec", 20.0);
        position_tolerance_ = get_parameter("position_tolerance").as_double();
        startup_warmup_sec_ = get_parameter("startup_warmup_sec").as_double();
        require_all_robots_ready_ = get_parameter("require_all_robots_ready").as_bool();
        failed_frontier_cooldown_sec_ =
            std::max(1.0, get_parameter("failed_frontier_cooldown_sec").as_double());
        
        // 初始化三个机器人的数据存储
        for (int i = 0; i < 3; ++i) {
            robot_frontiers_[i] = std::vector<fuzzy_msgs::msg::FuzzyPriority>();
            robot_sorted_frontiers_[i] = std::vector<fuzzy_msgs::msg::FuzzyPriority>();
            robot_selected_goal_[i] = std::nullopt;
            robot_ready_[i] = false;
            robot_failed_[i] = false;
        }
        
        // 创建三个订阅器，分别订阅三个机器人的优先级话题
        robot_0_sub_ = this->create_subscription<fuzzy_msgs::msg::FuzzyPriorityArray>(
            "/robot_0/frontier_priority_fuzzy", 10,
            std::bind(&MultiRobotGoalSelectorNode::robot0Callback, this, std::placeholders::_1));
        
        robot_1_sub_ = this->create_subscription<fuzzy_msgs::msg::FuzzyPriorityArray>(
            "/robot_1/frontier_priority_fuzzy", 10,
            std::bind(&MultiRobotGoalSelectorNode::robot1Callback, this, std::placeholders::_1));
        
        robot_2_sub_ = this->create_subscription<fuzzy_msgs::msg::FuzzyPriorityArray>(
            "/robot_2/frontier_priority_fuzzy", 10,
            std::bind(&MultiRobotGoalSelectorNode::robot2Callback, this, std::placeholders::_1));
        
        // 创建三个发布器，分别发布三个机器人的任务点
        robot_0_pub_ = this->create_publisher<fuzzy_msgs::msg::FuzzyPriority>(
            "/robot_0/task_point", 10);
        
        robot_1_pub_ = this->create_publisher<fuzzy_msgs::msg::FuzzyPriority>(
            "/robot_1/task_point", 10);
        
        robot_2_pub_ = this->create_publisher<fuzzy_msgs::msg::FuzzyPriority>(
            "/robot_2/task_point", 10);

        robot_0_failure_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/robot_0/planning_failure_frontier", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { planningFailureCallback(0, msg); });
        robot_1_failure_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/robot_1/planning_failure_frontier", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { planningFailureCallback(1, msg); });
        robot_2_failure_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/robot_2/planning_failure_frontier", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { planningFailureCallback(2, msg); });
        for (int i = 0; i < 3; ++i) {
            failed_state_subs_[i] = this->create_subscription<std_msgs::msg::Bool>(
                "/robot_" + std::to_string(i) + "/failed", rclcpp::QoS(1).reliable().transient_local(),
                [this, i](const std_msgs::msg::Bool::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    robot_failed_[i] = msg && msg->data;
                    if (robot_failed_[i]) {
                        robot_frontiers_[i].clear();
                        robot_sorted_frontiers_[i].clear();
                        robot_selected_goal_[i] = std::nullopt;
                        RCLCPP_WARN(this->get_logger(), "机器人 %d 已失效，停止参与 frontier 分配", i);
                    }
                });
        }

        conflict_count_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            "/goal_select/conflict_count", 10);
        
        RCLCPP_INFO(this->get_logger(), "Multi-Robot Goal Selector Node 已初始化");
        RCLCPP_INFO(this->get_logger(), "位置容差: %.2f 米", position_tolerance_);
        RCLCPP_INFO(this->get_logger(), "启动预热: require_all_robots_ready=%s, startup_warmup_sec=%.2f",
                    require_all_robots_ready_ ? "true" : "false", startup_warmup_sec_);
    }

private:
    // ROS订阅器和发布器
    rclcpp::Subscription<fuzzy_msgs::msg::FuzzyPriorityArray>::SharedPtr robot_0_sub_;
    rclcpp::Subscription<fuzzy_msgs::msg::FuzzyPriorityArray>::SharedPtr robot_1_sub_;
    rclcpp::Subscription<fuzzy_msgs::msg::FuzzyPriorityArray>::SharedPtr robot_2_sub_;
    
    rclcpp::Publisher<fuzzy_msgs::msg::FuzzyPriority>::SharedPtr robot_0_pub_;
    rclcpp::Publisher<fuzzy_msgs::msg::FuzzyPriority>::SharedPtr robot_1_pub_;
    rclcpp::Publisher<fuzzy_msgs::msg::FuzzyPriority>::SharedPtr robot_2_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr conflict_count_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_0_failure_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_1_failure_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_2_failure_sub_;
    std::array<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr, 3> failed_state_subs_;
    
    // 存储每个机器人的frontier数据
    std::array<std::vector<fuzzy_msgs::msg::FuzzyPriority>, 3> robot_frontiers_;
    
    // 存储每个机器人排序后的frontier数据
    std::array<std::vector<fuzzy_msgs::msg::FuzzyPriority>, 3> robot_sorted_frontiers_;
    
    // 存储每个机器人当前选中的目标
    std::array<std::optional<fuzzy_msgs::msg::FuzzyPriority>, 3> robot_selected_goal_;
    std::array<std::unordered_map<int, rclcpp::Time>, 3> failed_frontier_until_;
    
    // 参数
    double position_tolerance_;
    double startup_warmup_sec_{8.0};
    bool require_all_robots_ready_{true};
    double failed_frontier_cooldown_sec_{20.0};
    
    // 互斥锁保护共享数据
    std::mutex data_mutex_;

    std::array<bool, 3> robot_ready_{};
    std::array<bool, 3> robot_failed_{};
    bool warmup_started_{false};
    bool warmup_completed_{false};
    bool warmup_wait_logged_{false};
    std::chrono::steady_clock::time_point warmup_deadline_{};

    void planningFailureCallback(int robot_id, const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!msg || robot_id < 0 || robot_id >= 3 || msg->data < 0) {
            return;
        }

        std::lock_guard<std::mutex> lock(data_mutex_);
        if (robot_failed_[robot_id]) {
            return;
        }
        failed_frontier_until_[robot_id][msg->data] =
            this->now() + rclcpp::Duration::from_seconds(failed_frontier_cooldown_sec_);
        RCLCPP_WARN(
            this->get_logger(),
            "机器人 %d: frontier %d 因规划失败被临时跳过 %.1f 秒",
            robot_id,
            msg->data,
            failed_frontier_cooldown_sec_);
    }

    bool isFrontierTemporarilyBlocked(int robot_id, int frontier_id)
    {
        if (frontier_id < 0) {
            return false;
        }

        auto &blocked = failed_frontier_until_[robot_id];
        const auto now = this->now();
        for (auto it = blocked.begin(); it != blocked.end();) {
            if (it->second <= now) {
                it = blocked.erase(it);
            } else {
                ++it;
            }
        }

        return blocked.find(frontier_id) != blocked.end();
    }
    
    /**
     * @brief 机器人0的回调函数
     */
    void robot0Callback(const fuzzy_msgs::msg::FuzzyPriorityArray::SharedPtr msg)
    {
        processRobotFrontiers(0, msg);
    }
    
    /**
     * @brief 机器人1的回调函数
     */
    void robot1Callback(const fuzzy_msgs::msg::FuzzyPriorityArray::SharedPtr msg)
    {
        processRobotFrontiers(1, msg);
    }
    
    /**
     * @brief 机器人2的回调函数
     */
    void robot2Callback(const fuzzy_msgs::msg::FuzzyPriorityArray::SharedPtr msg)
    {
        processRobotFrontiers(2, msg);
    }
    
    /**
     * @brief 处理机器人接收到的frontier数据
     * @param robot_id 机器人ID (0, 1, 2)
     * @param msg 接收到的优先级消息
     */
    void processRobotFrontiers(int robot_id, const fuzzy_msgs::msg::FuzzyPriorityArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 清空该机器人之前的frontier数据
        robot_frontiers_[robot_id].clear();
        
        // 复制接收到的frontier数据
        for (const auto& priority : msg->priorities) {
            robot_frontiers_[robot_id].push_back(priority);
        }

        if (!robot_frontiers_[robot_id].empty() && !robot_ready_[robot_id]) {
            robot_ready_[robot_id] = true;
            RCLCPP_INFO(this->get_logger(), "机器人 %d 已收到首个非空 frontier 列表，count=%zu",
                        robot_id, robot_frontiers_[robot_id].size());
        }
        
        RCLCPP_DEBUG(this->get_logger(), "机器人 %d 收到 %zu 个frontier", 
                    robot_id, robot_frontiers_[robot_id].size());
        
        // 对该机器人的frontier进行排序（按分数降序）
        sortRobotFrontiers(robot_id);
        
        // 进行任务分配
        allocateTasks();
    }

    bool startupWarmupSatisfied()
    {
        if (warmup_completed_) {
            return true;
        }

        const auto now = std::chrono::steady_clock::now();

        if (!require_all_robots_ready_) {
            if (!warmup_started_) {
                warmup_started_ = true;
                warmup_deadline_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(startup_warmup_sec_));
                RCLCPP_INFO(this->get_logger(), "启动预热开始：%.2f 秒后允许派单", startup_warmup_sec_);
            }
        } else {
            bool all_ready = true;
            for (int i = 0; i < 3; ++i) {
                all_ready = all_ready && (robot_ready_[i] || robot_failed_[i]);
            }
            if (!all_ready) {
                if (!warmup_wait_logged_) {
                    RCLCPP_INFO(this->get_logger(), "等待三台机器人都收到非空 frontier 后再开始预热");
                    warmup_wait_logged_ = true;
                }
                return false;
            }

            if (!warmup_started_) {
                warmup_started_ = true;
                warmup_deadline_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(startup_warmup_sec_));
                RCLCPP_INFO(this->get_logger(),
                            "三台机器人已就绪，开始预热 %.2f 秒，预热结束后才允许派单",
                            startup_warmup_sec_);
            }
        }

        if (now < warmup_deadline_) {
            return false;
        }

        warmup_completed_ = true;
        RCLCPP_INFO(this->get_logger(), "启动预热完成，开始正常派单");
        return true;
    }
    
    /**
     * @brief 对指定机器人的frontier按分数排序
     * @param robot_id 机器人ID
     */
    void sortRobotFrontiers(int robot_id)
    {
        // 复制数据到排序容器
        robot_sorted_frontiers_[robot_id] = robot_frontiers_[robot_id];
        
        // 按分数降序排序
        std::sort(robot_sorted_frontiers_[robot_id].begin(), 
                  robot_sorted_frontiers_[robot_id].end(),
                  [](const fuzzy_msgs::msg::FuzzyPriority& a, 
                     const fuzzy_msgs::msg::FuzzyPriority& b) {
                      return a.total_score > b.total_score;
                  });
        
        // 调试输出排序结果
        if (!robot_sorted_frontiers_[robot_id].empty()) {
            RCLCPP_DEBUG(this->get_logger(), "机器人 %d 排序结果: 最高分 %.2f (ID: %d)", 
                        robot_id, 
                        robot_sorted_frontiers_[robot_id][0].total_score,
                        robot_sorted_frontiers_[robot_id][0].frontier_id);
        }
    }
    
    /**
     * @brief 计算两点之间的距离
     */
    double distanceBetween(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
    {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    /**
     * @brief 检查两点是否相同（在容差范围内）
     */
    bool isSamePosition(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
    {
        return distanceBetween(a, b) < position_tolerance_;
    }
    
    // 使用匈牙利算法最大化全体机器人的总前沿得分。
    void allocateTasks()
    {
        if (!startupWarmupSatisfied()) {
            return;
        }

        int conflict_count = 0;
        for (auto &selected_goal : robot_selected_goal_) {
            selected_goal = std::nullopt;
        }

        std::vector<int> active_robots;
        for (int i = 0; i < 3; ++i) {
            if (!robot_failed_[i] && !robot_sorted_frontiers_[i].empty()) {
                active_robots.push_back(i);
            } else {
                RCLCPP_WARN(this->get_logger(), "机器人 %d 没有有效的frontier数据", i);
            }
        }

        if (active_robots.empty()) {
            publishConflictCount(conflict_count);
            RCLCPP_WARN(this->get_logger(), "所有机器人都没有有效的frontier数据");
            return;
        }

        struct Candidate {
            int frontier_id;
            geometry_msgs::msg::Point position;
        };
        std::vector<Candidate> candidates;
        auto same_candidate = [this](const fuzzy_msgs::msg::FuzzyPriority &frontier,
                                     const Candidate &candidate) {
            return (frontier.frontier_id >= 0 && candidate.frontier_id >= 0 &&
                    frontier.frontier_id == candidate.frontier_id) ||
                   isSamePosition(frontier.global_position, candidate.position);
        };

        for (const int robot_id : active_robots) {
            for (const auto &frontier : robot_sorted_frontiers_[robot_id]) {
                if (isFrontierTemporarilyBlocked(robot_id, frontier.frontier_id)) {
                    continue;
                }
                const auto existing = std::find_if(
                    candidates.begin(), candidates.end(),
                    [&frontier, &same_candidate](const Candidate &candidate) {
                        return same_candidate(frontier, candidate);
                    });
                if (existing == candidates.end()) {
                    candidates.push_back({frontier.frontier_id, frontier.global_position});
                } else {
                    ++conflict_count;
                }
            }
        }

        if (candidates.empty()) {
            publishConflictCount(conflict_count);
            RCLCPP_WARN(this->get_logger(), "所有有效frontier都被临时屏蔽");
            return;
        }

        const double forbidden_utility = -1.0e9;
        const double unassigned_utility = -1.0e8;
        const size_t real_column_count = candidates.size();
        const size_t row_count = active_robots.size();
        const size_t column_count = real_column_count + row_count;
        std::vector<std::vector<double>> utility(
            row_count, std::vector<double>(column_count, unassigned_utility));
        std::vector<std::vector<std::optional<fuzzy_msgs::msg::FuzzyPriority>>> options(
            row_count,
            std::vector<std::optional<fuzzy_msgs::msg::FuzzyPriority>>(real_column_count));

        double max_utility = unassigned_utility;
        for (size_t row = 0; row < row_count; ++row) {
            const int robot_id = active_robots[row];
            for (size_t column = 0; column < real_column_count; ++column) {
                utility[row][column] = forbidden_utility;
                for (const auto &frontier : robot_sorted_frontiers_[robot_id]) {
                    if (isFrontierTemporarilyBlocked(robot_id, frontier.frontier_id) ||
                        !same_candidate(frontier, candidates[column])) {
                        continue;
                    }
                    if (!options[row][column].has_value() ||
                        frontier.total_score > options[row][column]->total_score) {
                        options[row][column] = frontier;
                        utility[row][column] = frontier.total_score;
                    }
                }
                if (options[row][column].has_value()) {
                    max_utility = std::max(max_utility, utility[row][column]);
                }
            }
        }

        // 标准匈牙利算法求解矩形最小费用问题；虚拟列表示机器人本轮不分配。
        std::vector<double> u(row_count + 1), v(column_count + 1);
        std::vector<size_t> p(column_count + 1), way(column_count + 1);
        for (size_t i = 1; i <= row_count; ++i) {
            p[0] = i;
            size_t j0 = 0;
            std::vector<double> minv(column_count + 1, std::numeric_limits<double>::infinity());
            std::vector<bool> used(column_count + 1, false);
            do {
                used[j0] = true;
                const size_t i0 = p[j0];
                double delta = std::numeric_limits<double>::infinity();
                size_t j1 = 0;
                for (size_t j = 1; j <= column_count; ++j) {
                    if (used[j]) {
                        continue;
                    }
                    const double cost = max_utility - utility[i0 - 1][j - 1];
                    const double cur = cost - u[i0] - v[j];
                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }
                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
                for (size_t j = 0; j <= column_count; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);
            do {
                const size_t j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        std::vector<size_t> assigned_column(row_count, column_count);
        for (size_t j = 1; j <= column_count; ++j) {
            if (p[j] != 0) {
                assigned_column[p[j] - 1] = j - 1;
            }
        }

        for (size_t row = 0; row < row_count; ++row) {
            const int robot_id = active_robots[row];
            const size_t column = assigned_column[row];
            if (column < real_column_count && options[row][column].has_value() &&
                utility[row][column] > forbidden_utility / 2.0) {
                robot_selected_goal_[robot_id] = options[row][column];
                const auto &goal = options[row][column].value();
                RCLCPP_INFO(
                    this->get_logger(),
                    "匈牙利分配: 机器人 %d -> frontier %d, 分数=%.2f, 位置=(%.2f, %.2f)",
                    robot_id, goal.frontier_id, goal.total_score,
                    goal.global_position.x, goal.global_position.y);
            } else {
                RCLCPP_WARN(this->get_logger(), "机器人 %d 本轮没有可分配frontier", robot_id);
            }
        }

        publishConflictCount(conflict_count);
        publishAllocatedGoals();
    }

    void publishConflictCount(int conflict_count)
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = std::max(0, conflict_count);
        conflict_count_pub_->publish(msg);
    }
    
    /**
     * @brief 发布分配好的目标给各个机器人
     */
    void publishAllocatedGoals()
    {
        // 创建发布器数组，方便循环
        std::array<rclcpp::Publisher<fuzzy_msgs::msg::FuzzyPriority>::SharedPtr, 3> publishers = {
            robot_0_pub_, robot_1_pub_, robot_2_pub_
        };
        
        // 发布每个机器人的目标
        for (int robot_id = 0; robot_id < 3; ++robot_id) {
            if (robot_selected_goal_[robot_id].has_value()) {
                const auto& goal = robot_selected_goal_[robot_id].value();
                publishers[robot_id]->publish(goal);
                
                RCLCPP_INFO(this->get_logger(), 
                           "发布给机器人 %d: frontier_id=%d, 分数=%.2f, 位置=(%.2f, %.2f)",
                           robot_id, goal.frontier_id, goal.total_score,
                           goal.global_position.x, goal.global_position.y);
            } else {
                RCLCPP_DEBUG(this->get_logger(), "机器人 %d 没有分配目标", robot_id);
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiRobotGoalSelectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
