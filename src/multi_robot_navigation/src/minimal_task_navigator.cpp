#include "rclcpp/rclcpp.hpp"
#include "frontier_msgs/msg/frontier_array_extended.hpp"
#include "fuzzy_msgs/msg/fuzzy_priority.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/exceptions.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <algorithm>
#include <functional>
#include <array>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "std_msgs/msg/bool.hpp"

class TaskPointNavigator : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    TaskPointNavigator()
    : Node("task_point_navigator"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        declare_parameter<std::string>("target_frame", "robot_0/map");
        declare_parameter<std::string>("local_map_frame", "robot_0/map");
        declare_parameter<std::string>("global_frame", "global_map");
        declare_parameter<std::string>("base_frame", "robot_0/base_link");
        declare_parameter<std::string>("global_map_topic", "/global_map");
        declare_parameter<std::string>("local_map_topic", "/robot_0/map");
        declare_parameter<double>("goal_obstacle_threshold", 65.0);
        declare_parameter<double>("local_known_threshold", 50.0);
        declare_parameter<double>("subgoal_spacing_m", 3.0);
        declare_parameter<double>("local_safety_margin_m", 0.30);
        declare_parameter<double>("astar_unknown_cost_multiplier", 100.0);
        declare_parameter<int>("astar_max_expansions", 150000);
        declare_parameter<int>("max_replan_retries", 2);
        declare_parameter<bool>("enable_explore_replanning", false);
        declare_parameter<double>("progress_timeout_sec", 12.0);
        declare_parameter<double>("min_progress_delta_m", 0.15);
        declare_parameter<double>("final_goal_tolerance_m", 0.50);
        declare_parameter<double>("tf_timeout_sec", 0.30);
        declare_parameter<double>("fallback_subgoal_distance_m", 2.0);
        declare_parameter<int>("repulsion_window_radius_cells", 3);
        declare_parameter<double>("repulsion_max_shift_m", 0.15);
        declare_parameter<std::string>("exploration_stop_topic", "/exploration_stop");
        declare_parameter<bool>("enable_low_battery_handling", true);
        declare_parameter<double>("low_battery_check_threshold", 0.25);
        declare_parameter<double>("low_battery_threshold", 0.25);
        declare_parameter<double>("home_x", 0.0);
        declare_parameter<double>("home_y", 0.0);
        declare_parameter<std::string>("battery_reset_service", "reset_battery");
        declare_parameter<std::string>("battery_discharge_service", "set_discharge_enabled");
        declare_parameter<std::string>("low_battery_metrics_topic", "low_battery_metrics");
        declare_parameter<std::string>("task_metrics_topic", "task_metrics");
        declare_parameter<std::string>("planning_failure_topic", "planning_failure_frontier");
        declare_parameter<bool>("enable_goal_settle_wait", true);
        declare_parameter<double>("goal_settle_wait_sec", 4.0);
        declare_parameter<bool>("enable_frontier_disappearance_switch", false);
        declare_parameter<std::string>("frontier_topic", "/frontiers_info");
        declare_parameter<double>("frontier_missing_hold_sec", 2.0);
        declare_parameter<double>("frontier_position_tolerance_m", 1.0);
        declare_parameter<double>("frontier_disappearance_ignore_near_goal_m", 3.0);

        target_frame_ = get_parameter("target_frame").as_string();
        local_map_frame_ = get_parameter("local_map_frame").as_string();
        global_frame_ = get_parameter("global_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        global_map_topic_ = get_parameter("global_map_topic").as_string();
        local_map_topic_ = get_parameter("local_map_topic").as_string();
        goal_obstacle_threshold_ = get_parameter("goal_obstacle_threshold").as_double();
        local_known_threshold_ = get_parameter("local_known_threshold").as_double();
        subgoal_spacing_m_ = std::max(0.2, get_parameter("subgoal_spacing_m").as_double());
        local_safety_margin_m_ = std::max(0.0, get_parameter("local_safety_margin_m").as_double());
        astar_unknown_cost_multiplier_ = std::max(1.0, get_parameter("astar_unknown_cost_multiplier").as_double());
        stop_topic_ = get_parameter("exploration_stop_topic").as_string();
        astar_max_expansions_ = static_cast<int>(
            std::max<int64_t>(1000, get_parameter("astar_max_expansions").as_int()));
        max_replan_retries_ = static_cast<int>(
            std::max<int64_t>(0, get_parameter("max_replan_retries").as_int()));
        enable_explore_replanning_ = get_parameter("enable_explore_replanning").as_bool();
        progress_timeout_sec_ = std::max(2.0, get_parameter("progress_timeout_sec").as_double());
        min_progress_delta_m_ = std::max(0.01, get_parameter("min_progress_delta_m").as_double());
        final_goal_tolerance_m_ = std::max(0.05, get_parameter("final_goal_tolerance_m").as_double());
        tf_timeout_sec_ = std::max(0.05, get_parameter("tf_timeout_sec").as_double());
        fallback_subgoal_distance_m_ = std::max(0.2, get_parameter("fallback_subgoal_distance_m").as_double());
        repulsion_window_radius_cells_ = static_cast<int>(
            std::max<int64_t>(1, get_parameter("repulsion_window_radius_cells").as_int()));
        repulsion_max_shift_m_ = std::max(0.0, get_parameter("repulsion_max_shift_m").as_double());
        enable_low_battery_handling_ = get_parameter("enable_low_battery_handling").as_bool();
        low_battery_check_threshold_ = std::clamp(
            get_parameter("low_battery_check_threshold").as_double(), 0.0, 1.0);
        low_battery_threshold_ = std::clamp(
            get_parameter("low_battery_threshold").as_double(), 0.0, 1.0);
        if (low_battery_check_threshold_ < low_battery_threshold_) {
            low_battery_check_threshold_ = low_battery_threshold_;
        }
        home_x_ = get_parameter("home_x").as_double();
        home_y_ = get_parameter("home_y").as_double();
        battery_reset_service_ = get_parameter("battery_reset_service").as_string();
        battery_discharge_service_ = get_parameter("battery_discharge_service").as_string();
        low_battery_metrics_topic_ = get_parameter("low_battery_metrics_topic").as_string();
        task_metrics_topic_ = get_parameter("task_metrics_topic").as_string();
        planning_failure_topic_ = get_parameter("planning_failure_topic").as_string();
        enable_goal_settle_wait_ = get_parameter("enable_goal_settle_wait").as_bool();
        goal_settle_wait_sec_ = std::max(0.0, get_parameter("goal_settle_wait_sec").as_double());
        enable_frontier_disappearance_switch_ =
            get_parameter("enable_frontier_disappearance_switch").as_bool();
        frontier_topic_ = get_parameter("frontier_topic").as_string();
        frontier_missing_hold_sec_ = std::max(
            0.5, get_parameter("frontier_missing_hold_sec").as_double());
        frontier_position_tolerance_m_ = std::max(
            0.1, get_parameter("frontier_position_tolerance_m").as_double());
        frontier_disappearance_ignore_near_goal_m_ = std::max(
            0.0, get_parameter("frontier_disappearance_ignore_near_goal_m").as_double());

        task_point_sub_ = this->create_subscription<fuzzy_msgs::msg::FuzzyPriority>(
            "/task_point", 10,
            std::bind(&TaskPointNavigator::taskPointCallback, this, std::placeholders::_1));

        battery_state_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery_state", 10,
            std::bind(&TaskPointNavigator::batteryStateCallback, this, std::placeholders::_1));
        frontier_sub_ = this->create_subscription<frontier_msgs::msg::FrontierArrayExtended>(
            frontier_topic_, 10,
            std::bind(&TaskPointNavigator::frontierCallback, this, std::placeholders::_1));

        rclcpp::QoS map_qos(rclcpp::KeepLast(1));
        map_qos.reliable();
        map_qos.durability_volatile();

        global_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            global_map_topic_, map_qos,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(global_map_mutex_);
                global_map_ = msg;
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), *this->get_clock(), 3000,
                    "已收到 global_map: frame=%s size=%u x %u res=%.3f",
                    msg->header.frame_id.c_str(),
                    msg->info.width, msg->info.height, msg->info.resolution);
            });

        local_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            local_map_topic_, map_qos,
            [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(local_map_mutex_);
                local_map_ = msg;
            });

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "task_goal_markers", rclcpp::QoS(1).transient_local().reliable());
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        low_battery_metrics_pub_ = this->create_publisher<std_msgs::msg::String>(
            low_battery_metrics_topic_, rclcpp::QoS(10).reliable());
        task_metrics_pub_ = this->create_publisher<std_msgs::msg::String>(
            task_metrics_topic_, rclcpp::QoS(20).reliable());
        planning_failure_pub_ = this->create_publisher<std_msgs::msg::Int32>(
            planning_failure_topic_, rclcpp::QoS(10).reliable());
        battery_reset_client_ = this->create_client<std_srvs::srv::Trigger>(battery_reset_service_);
        battery_discharge_client_ = this->create_client<std_srvs::srv::SetBool>(battery_discharge_service_);

        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        waitForNav2Server();

        rclcpp::QoS stop_qos(rclcpp::KeepLast(1));
        stop_qos.reliable();
        stop_qos.transient_local();

        exploration_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            stop_topic_, stop_qos,
            std::bind(&TaskPointNavigator::explorationStopCallback, this, std::placeholders::_1));
        progress_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TaskPointNavigator::progressTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Task Point Navigator 已启动");
        RCLCPP_INFO(this->get_logger(), "target_frame=%s local_map_frame=%s global_frame=%s base_frame=%s",
                    target_frame_.c_str(), local_map_frame_.c_str(), global_frame_.c_str(), base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "global_map_topic=%s local_map_topic=%s",
                    global_map_topic_.c_str(), local_map_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "RViz MarkerArray topic=%s",
                    (this->get_namespace() + std::string("/task_goal_markers")).c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "low_battery: enabled=%s check<=%.2f trigger<=%.2f home=(%.2f, %.2f) reset_service=%s discharge_service=%s",
            enable_low_battery_handling_ ? "true" : "false",
            low_battery_check_threshold_,
            low_battery_threshold_,
            home_x_, home_y_,
            battery_reset_service_.c_str(),
            battery_discharge_service_.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "frontier disappearance switch: enabled=%s topic=%s hold=%.1fs position_tolerance=%.2fm",
            enable_frontier_disappearance_switch_ ? "true" : "false",
            frontier_topic_.c_str(),
            frontier_missing_hold_sec_,
            frontier_position_tolerance_m_);

        publishLowBatteryMetrics("startup");

        
    }

private:
    rclcpp::Subscription<fuzzy_msgs::msg::FuzzyPriority>::SharedPtr task_point_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_sub_;
    rclcpp::Subscription<frontier_msgs::msg::FrontierArrayExtended>::SharedPtr frontier_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr low_battery_metrics_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_metrics_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr planning_failure_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr battery_reset_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr battery_discharge_client_;
    GoalHandleNavigate::SharedPtr current_goal_handle_;
    rclcpp::TimerBase::SharedPtr progress_timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_stop_sub_;
    std::string stop_topic_;
    bool exploration_stopped_{false};

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex state_mutex_;
    mutable std::mutex global_map_mutex_;
    std::mutex local_map_mutex_;

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr local_map_;

    std::string target_frame_;
    std::string local_map_frame_;
    std::string global_frame_;
    std::string base_frame_;
    std::string global_map_topic_;
    std::string local_map_topic_;

    double goal_obstacle_threshold_{65.0};
    double local_known_threshold_{50.0};
    double subgoal_spacing_m_{1.0};
    double local_safety_margin_m_{0.30};
    double astar_unknown_cost_multiplier_{4.0};
    int astar_max_expansions_{120000};
    int max_replan_retries_{2};
    bool enable_explore_replanning_{false};
    double progress_timeout_sec_{12.0};
    double min_progress_delta_m_{0.15};
    double final_goal_tolerance_m_{0.50};
    double tf_timeout_sec_{0.30};
    double fallback_subgoal_distance_m_{2.0};
    int repulsion_window_radius_cells_{3};
    double repulsion_max_shift_m_{0.15};
    bool enable_low_battery_handling_{true};
    double low_battery_check_threshold_{0.5};
    double low_battery_threshold_{0.2};
    double home_x_{0.0};
    double home_y_{0.0};
    std::string battery_reset_service_;
    std::string battery_discharge_service_;
    std::string low_battery_metrics_topic_;
    std::string task_metrics_topic_;
    std::string planning_failure_topic_;
    std::string frontier_topic_;
    bool enable_goal_settle_wait_{true};
    double goal_settle_wait_sec_{4.0};
    bool enable_frontier_disappearance_switch_{false};
    double frontier_missing_hold_sec_{2.0};
    double frontier_position_tolerance_m_{1.0};
    double frontier_disappearance_ignore_near_goal_m_{3.0};
    bool low_battery_handling_active_{false};
    bool returning_home_active_{false};
    bool charging_active_{false};
    bool pending_return_home_plan_{false};
    int return_home_retry_count_{0};
    int max_return_home_retry_count_{200};
    double low_battery_mechanism_time_sec_{0.0};
    double low_battery_return_distance_total_m_{0.0};
    int low_battery_charge_count_{0};
    double goal_settle_wait_time_sec_{0.0};
    double last_low_battery_return_distance_m_{0.0};
    double last_low_battery_return_time_sec_{0.0};
    double last_low_battery_percentage_{1.0};
    std::string last_low_battery_event_{"startup"};

    bool task_active_{false};
    bool nav_goal_active_{false};
    bool cancel_requested_for_replan_{false};
    bool cancel_requested_for_frontier_disappearance_{false};
    bool goal_settle_wait_active_{false};
    int task_counter_{0};
    rclcpp::Time goal_settle_wait_start_{0, 0, RCL_ROS_TIME};
    rclcpp::Time goal_settle_wait_deadline_{0, 0, RCL_ROS_TIME};
    rclcpp::Time frontier_missing_since_{0, 0, RCL_ROS_TIME};

    static std::string escapeJsonString(const std::string &input)
    {
        std::ostringstream oss;
        for (const char ch : input) {
            switch (ch) {
                case '\\':
                    oss << "\\\\";
                    break;
                case '"':
                    oss << "\\\"";
                    break;
                case '\n':
                    oss << "\\n";
                    break;
                case '\r':
                    oss << "\\r";
                    break;
                case '\t':
                    oss << "\\t";
                    break;
                default:
                    oss << ch;
                    break;
            }
        }
        return oss.str();
    }

    void publishTaskEvent(const std::string &event_name, const std::string &detail = "")
    {
        if (!task_metrics_pub_) {
            return;
        }

        int frontier_id = -1;
        int task_counter = 0;
        double score = 0.0;
        double last_battery_percentage = 1.0;
        bool task_active = false;
        bool nav_goal_active = false;
        bool low_battery_active = false;
        bool returning_home = false;
        bool charging = false;
        geometry_msgs::msg::Point final_goal;
        geometry_msgs::msg::Point adjusted_goal;
        geometry_msgs::msg::Point current_subgoal;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            frontier_id = current_frontier_id_;
            task_counter = task_counter_;
            score = current_score_;
            last_battery_percentage = last_low_battery_percentage_;
            task_active = task_active_;
            nav_goal_active = nav_goal_active_;
            low_battery_active = low_battery_handling_active_;
            returning_home = returning_home_active_;
            charging = charging_active_;
            final_goal = final_goal_global_;
            adjusted_goal = adjusted_goal_global_;
            current_subgoal = current_subgoal_global_;
        }

        const std::string phase = charging
                                      ? "charging"
                                      : (returning_home
                                             ? "return_home"
                                             : (low_battery_active ? "low_battery_flow" : "explore"));
        const std::string goal_type = returning_home ? "home" : "frontier";

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        oss << "{"
            << "\"event\":\"" << escapeJsonString(event_name) << "\","
            << "\"detail\":\"" << escapeJsonString(detail) << "\","
            << "\"namespace\":\"" << escapeJsonString(this->get_namespace()) << "\","
            << "\"timestamp_unix\":" << static_cast<double>(std::time(nullptr)) << ","
            << "\"task_counter\":" << task_counter << ","
            << "\"frontier_id\":" << frontier_id << ","
            << "\"score\":" << score << ","
            << "\"phase\":\"" << phase << "\","
            << "\"goal_type\":\"" << goal_type << "\","
            << "\"last_battery_percentage\":" << last_battery_percentage << ","
            << "\"task_active\":" << (task_active ? "true" : "false") << ","
            << "\"nav_goal_active\":" << (nav_goal_active ? "true" : "false") << ","
            << "\"low_battery_handling_active\":" << (low_battery_active ? "true" : "false") << ","
            << "\"returning_home_active\":" << (returning_home ? "true" : "false") << ","
            << "\"charging_active\":" << (charging ? "true" : "false") << ","
            << "\"final_goal\":{\"x\":" << final_goal.x << ",\"y\":" << final_goal.y << "},"
            << "\"adjusted_goal\":{\"x\":" << adjusted_goal.x << ",\"y\":" << adjusted_goal.y << "},"
            << "\"current_subgoal\":{\"x\":" << current_subgoal.x << ",\"y\":" << current_subgoal.y << "}"
            << "}";

        std_msgs::msg::String msg;
        msg.data = oss.str();
        task_metrics_pub_->publish(msg);
    }
    int replan_retry_count_{0};
    int current_frontier_id_{-1};
    double current_score_{0.0};
    geometry_msgs::msg::Point final_goal_global_{};
    geometry_msgs::msg::Point adjusted_goal_global_{};
    geometry_msgs::msg::Point current_subgoal_global_{};
    std::vector<geometry_msgs::msg::Point> global_path_;
    size_t current_waypoint_index_{0};
    rclcpp::Time last_progress_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_feedback_time_{0, 0, RCL_ROS_TIME};
    double best_remaining_distance_{std::numeric_limits<double>::infinity()};

    void waitForNav2Server()
    {
        while (rclcpp::ok() && !nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "等待 Nav2 服务器启动...");
        }
        RCLCPP_INFO(this->get_logger(), "Nav2 服务器已就绪");
    }

    static double pointDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
    {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    void logEnergyStateTransition(
        const std::string &from_state,
        const std::string &to_state,
        const std::string &detail) const
    {
        RCLCPP_WARN(
            this->get_logger(),
            "能量状态切换: %s -> %s (%s)",
            from_state.c_str(),
            to_state.c_str(),
            detail.c_str());
    }


    struct VisualizationSnapshot
    {
        bool task_active{false};
        int frontier_id{-1};
        geometry_msgs::msg::Point final_goal{};
        geometry_msgs::msg::Point adjusted_goal{};
        geometry_msgs::msg::Point current_subgoal{};
        std::vector<geometry_msgs::msg::Point> global_path;
        size_t current_waypoint_index{0};
    };

    void explorationStopCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        GoalHandleNavigate::SharedPtr goal_handle;
        bool stopping = false;
        bool should_resume_discharge = false;
        bool keep_low_battery_flow = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            if (exploration_stopped_ == msg->data) {
                return;
            }

            exploration_stopped_ = msg->data;

            if (!exploration_stopped_) {
                RCLCPP_INFO(this->get_logger(), "收到探索恢复信号");
                return;
            }

            keep_low_battery_flow =
                low_battery_handling_active_ || returning_home_active_ || charging_active_ || pending_return_home_plan_;
            if (keep_low_battery_flow) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "收到探索停止信号，但机器人处于低电量返航/充电流程中，保持当前保护状态直到流程结束");
                return;
            }

            should_resume_discharge = low_battery_handling_active_ || returning_home_active_ || charging_active_;
            low_battery_handling_active_ = false;
            returning_home_active_ = false;
            charging_active_ = false;
            pending_return_home_plan_ = false;
            stopping = true;
            goal_handle = current_goal_handle_;
        }

        if (!stopping) {
            return;
        }

        RCLCPP_WARN(this->get_logger(), "收到探索停止信号，取消当前导航并忽略后续任务");
        clearTaskState("exploration stopped by supervisor");
        if (should_resume_discharge) {
            setBatteryDischargeEnabled(true, "exploration stopped");
        }

        if (goal_handle) {
            nav2_client_->async_cancel_goal(goal_handle);
        }
    }

    VisualizationSnapshot makeVisualizationSnapshot()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        VisualizationSnapshot snapshot;
        snapshot.task_active = task_active_;
        snapshot.frontier_id = current_frontier_id_;
        snapshot.final_goal = final_goal_global_;
        snapshot.adjusted_goal = adjusted_goal_global_;
        snapshot.current_subgoal = current_subgoal_global_;
        snapshot.global_path = global_path_;
        snapshot.current_waypoint_index = current_waypoint_index_;
        return snapshot;
    }

    visualization_msgs::msg::Marker makeSphereMarker(
        int id, const std::string &ns,
        const geometry_msgs::msg::Point &point,
        double scale,
        float r, float g, float b, float a) const
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = global_frame_;
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = scale;
        marker.scale.y = scale;
        marker.scale.z = scale;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        return marker;
    }

    visualization_msgs::msg::Marker makeTextMarker(
        int id, const std::string &ns,
        const geometry_msgs::msg::Point &point,
        const std::string &text,
        double scale_z,
        float r, float g, float b, float a) const
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = global_frame_;
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = point;
        marker.pose.position.z += 0.45;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = scale_z;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        marker.text = text;
        return marker;
    }

    visualization_msgs::msg::Marker makePathMarker(
        int id, const std::string &ns,
        const std::vector<geometry_msgs::msg::Point> &points,
        float r, float g, float b, float a) const
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = global_frame_;
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.08;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = a;
        marker.points = points;
        return marker;
    }

    void publishVisualization()
    {
        if (!marker_pub_) {
            return;
        }

        const auto snapshot = makeVisualizationSnapshot();
        visualization_msgs::msg::MarkerArray marker_array;

        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);

        if (!snapshot.task_active) {
            marker_pub_->publish(marker_array);
            return;
        }

        marker_array.markers.push_back(
            makeSphereMarker(0, "final_goal", snapshot.final_goal, 0.28, 1.0f, 0.0f, 0.0f, 0.95f));
        marker_array.markers.push_back(
            makeTextMarker(1, "final_goal_text", snapshot.final_goal,
                           "final goal id=" + std::to_string(snapshot.frontier_id), 0.22,
                           1.0f, 0.2f, 0.2f, 1.0f));

        marker_array.markers.push_back(
            makeSphereMarker(2, "adjusted_goal", snapshot.adjusted_goal, 0.24, 1.0f, 0.8f, 0.0f, 0.95f));
        marker_array.markers.push_back(
            makeTextMarker(3, "adjusted_goal_text", snapshot.adjusted_goal, "adjusted goal", 0.20,
                           1.0f, 0.85f, 0.2f, 1.0f));

        if (!snapshot.global_path.empty()) {
            marker_array.markers.push_back(
                makePathMarker(4, "global_path", snapshot.global_path, 0.2f, 0.6f, 1.0f, 0.9f));
        }

        if (snapshot.current_waypoint_index < snapshot.global_path.size()) {
            marker_array.markers.push_back(
                makeSphereMarker(5, "current_subgoal", snapshot.current_subgoal, 0.22, 0.0f, 1.0f, 0.0f, 0.95f));
            marker_array.markers.push_back(
                makeTextMarker(6, "current_subgoal_text", snapshot.current_subgoal,
                               "subgoal #" + std::to_string(snapshot.current_waypoint_index), 0.20,
                               0.2f, 1.0f, 0.2f, 1.0f));
        }

        marker_pub_->publish(marker_array);
    }

    void taskPointCallback(const fuzzy_msgs::msg::FuzzyPriority::SharedPtr msg)
    {
        if (!std::isfinite(msg->global_position.x) || !std::isfinite(msg->global_position.y)) {
            RCLCPP_ERROR(this->get_logger(), "收到无效 global_position，忽略任务");
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);

            if (exploration_stopped_) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 3000,
                    "探索已停止，忽略新任务 frontier_id=%d", msg->frontier_id);
                return;
            }

            if (low_battery_handling_active_ || returning_home_active_ || charging_active_ || pending_return_home_plan_) {
                std::string phase = "low_battery_flow";
                if (charging_active_) {
                    phase = "charging";
                } else if (returning_home_active_) {
                    phase = "return_home";
                } else if (pending_return_home_plan_) {
                    phase = "return_home_pending";
                }
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 3000,
                    "机器人处于低电量流程(%s)中，忽略新任务 frontier_id=%d",
                    phase.c_str(), msg->frontier_id);
                return;
            }

            if (goal_settle_wait_active_) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 3000,
                    "机器人正在到点静置等待中，忽略新任务 frontier_id=%d",
                    msg->frontier_id);
                return;
            }

            if (task_active_) {
                RCLCPP_WARN(this->get_logger(), "当前仍在执行任务 %d，忽略新任务 %d",
                            current_frontier_id_, msg->frontier_id);
                return;
            }

            task_counter_++;
            task_active_ = true;
            nav_goal_active_ = false;
            cancel_requested_for_replan_ = false;
            cancel_requested_for_frontier_disappearance_ = false;
            replan_retry_count_ = 0;
            current_frontier_id_ = msg->frontier_id;
            current_score_ = msg->total_score;
            final_goal_global_ = msg->global_position;
            adjusted_goal_global_ = msg->global_position;
            current_subgoal_global_ = geometry_msgs::msg::Point{};
            global_path_.clear();
            current_waypoint_index_ = 0;
            current_goal_handle_.reset();
            best_remaining_distance_ = std::numeric_limits<double>::infinity();
            last_progress_time_ = this->now();
            last_feedback_time_ = this->now();
            frontier_missing_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }

        RCLCPP_INFO(this->get_logger(),
                    "收到任务 #%d: frontier_id=%d score=%.3f global_goal=(%.2f, %.2f)",
                    task_counter_, msg->frontier_id, msg->total_score,
                    msg->global_position.x, msg->global_position.y);
        publishTaskEvent("task_received", "accepted new frontier task");

        publishVisualization();

        if (!planAndDispatch(false, "new task")) {
            publishTaskEvent("task_plan_failed", "initial planning failed");
            clearTaskState("initial planning failed");
        }
    }

    void frontierCallback(const frontier_msgs::msg::FrontierArrayExtended::SharedPtr msg)
    {
        if (!enable_frontier_disappearance_switch_ || !msg) {
            return;
        }

        GoalHandleNavigate::SharedPtr goal_handle;
        int vanished_frontier_id = -1;
        bool should_cancel = false;
        bool should_clear_without_cancel = false;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!task_active_ || current_frontier_id_ < 0 || exploration_stopped_ ||
                low_battery_handling_active_ || returning_home_active_ || charging_active_ ||
                pending_return_home_plan_ || goal_settle_wait_active_ ||
                cancel_requested_for_frontier_disappearance_) {
                frontier_missing_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
                return;
            }

            bool current_frontier_exists = false;
            for (const auto &frontier : msg->frontiers_extended) {
                const bool same_id = frontier.id == current_frontier_id_;
                const double dx = frontier.position[0] - final_goal_global_.x;
                const double dy = frontier.position[1] - final_goal_global_.y;
                const bool same_area =
                    std::hypot(dx, dy) <= frontier_position_tolerance_m_;
                if (same_id || same_area) {
                    current_frontier_exists = true;
                    break;
                }
            }

            if (current_frontier_exists) {
                frontier_missing_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
                return;
            }

            double robot_x = 0.0;
            double robot_y = 0.0;
            if (getRobotPoseInFrame(global_frame_, robot_x, robot_y) &&
                std::hypot(robot_x - final_goal_global_.x, robot_y - final_goal_global_.y) <=
                    frontier_disappearance_ignore_near_goal_m_) {
                frontier_missing_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
                return;
            }

            const auto now = this->now();
            if (frontier_missing_since_.nanoseconds() == 0) {
                frontier_missing_since_ = now;
                return;
            }
            if ((now - frontier_missing_since_).seconds() < frontier_missing_hold_sec_) {
                return;
            }

            vanished_frontier_id = current_frontier_id_;
            cancel_requested_for_frontier_disappearance_ = true;
            goal_handle = current_goal_handle_;
            should_cancel = static_cast<bool>(goal_handle);
            should_clear_without_cancel = !should_cancel;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "当前目标 frontier_id=%d 已连续 %.1f 秒不在最新前沿列表中，取消旧任务并等待重新分配",
            vanished_frontier_id,
            frontier_missing_hold_sec_);
        publishTaskEvent(
            "target_frontier_disappeared",
            "current frontier disappeared from latest frontier list");

        if (should_cancel) {
            nav2_client_->async_cancel_goal(goal_handle);
        } else if (should_clear_without_cancel) {
            clearTaskState("target frontier disappeared");
        }
    }

    void progressTimerCallback()
    {
        bool settle_completed = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (goal_settle_wait_active_ && this->now() >= goal_settle_wait_deadline_) {
                const double waited = std::max(0.0, (this->now() - goal_settle_wait_start_).seconds());
                goal_settle_wait_time_sec_ += waited;
                goal_settle_wait_active_ = false;
                goal_settle_wait_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
                goal_settle_wait_deadline_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
                settle_completed = true;
            }
        }

        if (settle_completed) {
            publishTaskEvent("task_completed", "final waypoint reached after settle wait");
            publishTaskEvent("task_settle_wait_finished", "goal settle wait completed");
            publishLowBatteryMetrics("goal_settle_wait_finished");
            setBatteryDischargeEnabled(true, "goal settle wait finished");
            clearTaskState("final waypoint reached after settle wait");
            return;
        }

        if (!enable_explore_replanning_) {
            return;
        }

        bool should_cancel = false;
        int frontier_id = -1;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!task_active_ || !nav_goal_active_ || cancel_requested_for_replan_ ||
                low_battery_handling_active_ || returning_home_active_ || charging_active_ || pending_return_home_plan_) {
                return;
            }
            const auto now = this->now();
            if ((now - last_progress_time_).seconds() > progress_timeout_sec_) {
                cancel_requested_for_replan_ = true;
                should_cancel = true;
                frontier_id = current_frontier_id_;
            }
        }

        if (!should_cancel) {
            return;
        }

        RCLCPP_WARN(this->get_logger(),
                    "任务 %d 在 %.1f 秒内无明显进展，取消当前子目标并重规划",
                    frontier_id, progress_timeout_sec_);

        GoalHandleNavigate::SharedPtr goal_handle;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            goal_handle = current_goal_handle_;
        }

        if (goal_handle) {
            nav2_client_->async_cancel_goal(goal_handle);
        } else {
            if (!replanCurrentTask("progress timeout without goal handle")) {
                clearTaskState("replan failed after timeout");
            }
        }
    }

    void clearTaskState(const std::string &reason)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (task_active_) {
                RCLCPP_WARN(this->get_logger(), "清理任务状态: frontier_id=%d, reason=%s",
                            current_frontier_id_, reason.c_str());
            }
            task_active_ = false;
            nav_goal_active_ = false;
            cancel_requested_for_replan_ = false;
            cancel_requested_for_frontier_disappearance_ = false;
            goal_settle_wait_active_ = false;
            replan_retry_count_ = 0;
            current_frontier_id_ = -1;
            current_score_ = 0.0;
            final_goal_global_ = geometry_msgs::msg::Point{};
            adjusted_goal_global_ = geometry_msgs::msg::Point{};
            current_subgoal_global_ = geometry_msgs::msg::Point{};
            global_path_.clear();
            current_waypoint_index_ = 0;
            best_remaining_distance_ = std::numeric_limits<double>::infinity();
            current_goal_handle_.reset();
            last_progress_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            last_feedback_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            goal_settle_wait_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            goal_settle_wait_deadline_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            frontier_missing_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        }
        publishVisualization();
    }

    void batteryStateCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        if (!enable_low_battery_handling_ || !msg) {
            return;
        }

        if (!msg->present || !std::isfinite(msg->percentage)) {
            return;
        }

        const double battery_percentage =
            std::clamp(static_cast<double>(msg->percentage), 0.0, 1.0);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (exploration_stopped_ || low_battery_handling_active_) {
                return;
            }
            if (battery_percentage > low_battery_check_threshold_) {
                return;
            }
            if (battery_percentage > low_battery_threshold_) {
                return;
            }

            low_battery_handling_active_ = true;
            returning_home_active_ = true;
            charging_active_ = false;
            pending_return_home_plan_ = true;
            return_home_retry_count_ = 0;
            last_low_battery_percentage_ = battery_percentage;
            last_low_battery_event_ = "triggered";
        }

        triggerReturnHome(battery_percentage);
    }

    void triggerReturnHome(double battery_percentage)
    {
        double robot_x = home_x_;
        double robot_y = home_y_;
        const bool pose_ok = getRobotPoseInFrame(global_frame_, robot_x, robot_y);
        if (!pose_ok) {
            RCLCPP_WARN(
                this->get_logger(),
                "低电量触发时获取机器人位姿失败，返航距离退化为欧氏距离估计");
        }

        const double estimated_return_distance_m =
            estimateReturnDistanceMeters(robot_x, robot_y, home_x_, home_y_);
        const double estimated_return_time_sec =
            std::max(0.0, estimated_return_distance_m / std::max(0.01, 0.35));

        GoalHandleNavigate::SharedPtr goal_handle;
        int frontier_id = -1;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            goal_handle = current_goal_handle_;
            frontier_id = current_frontier_id_;
            last_low_battery_return_distance_m_ = estimated_return_distance_m;
            last_low_battery_return_time_sec_ = estimated_return_time_sec;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "低电量触发返航: battery=%.3f trigger=%.3f home=(%.2f, %.2f) frontier_id=%d estimated_return_dist=%.2fm",
            battery_percentage,
            low_battery_threshold_,
            home_x_,
            home_y_,
            frontier_id,
            estimated_return_distance_m);
        logEnergyStateTransition(
            "explore",
            "return_home",
            "battery below threshold, cancel current task and go home");

        setBatteryDischargeEnabled(false, "low battery return-home");
        publishTaskEvent("return_home_triggered", "battery below threshold, cancel current task and go home");
        publishLowBatteryMetrics("triggered");
        publishZeroVelocity();

        if (goal_handle) {
            nav2_client_->async_cancel_goal(goal_handle);
            return;
        }

        clearTaskState("low battery switch to return-home");
        if (!startReturnHomeTask()) {
            publishTaskEvent("return_home_dispatch_failed", "failed to plan initial return-home task");
            publishLowBatteryMetrics("return_home_dispatch_failed");
            publishZeroVelocity();
        }
    }

    bool startReturnHomeTask()
    {
        int retry_count = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (return_home_retry_count_ >= max_return_home_retry_count_) {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "返航重试次数已达到上限 %d，保持低电保护并停止重新派发",
                    max_return_home_retry_count_);
                return false;
            }
            ++return_home_retry_count_;
            retry_count = return_home_retry_count_;
            task_active_ = true;
            nav_goal_active_ = false;
            cancel_requested_for_replan_ = false;
            replan_retry_count_ = 0;
            current_frontier_id_ = -100;
            current_score_ = 0.0;
            final_goal_global_.x = home_x_;
            final_goal_global_.y = home_y_;
            final_goal_global_.z = 0.0;
            adjusted_goal_global_ = final_goal_global_;
            current_subgoal_global_ = geometry_msgs::msg::Point{};
            global_path_.clear();
            current_waypoint_index_ = 0;
            current_goal_handle_.reset();
            best_remaining_distance_ = std::numeric_limits<double>::infinity();
            last_progress_time_ = this->now();
            last_feedback_time_ = this->now();
        }

        publishVisualization();
        RCLCPP_INFO(
            this->get_logger(),
            "切换为返航任务: home=(%.2f, %.2f), retry=%d, 返航期间忽略探索任务与普通超时重规划",
            home_x_, home_y_, retry_count);
        const bool dispatched = planAndDispatch(false, "return_home");
        publishTaskEvent(
            dispatched ? "return_home_started" : "return_home_dispatch_failed",
            dispatched ? "home target dispatched" : "failed to plan return-home task");
        return dispatched;
    }

    void requestBatteryRecharge()
    {
        if (!batteryResetReady()) {
            RCLCPP_ERROR(this->get_logger(), "reset_battery 服务不可用，无法执行充电恢复");
            resetLowBatteryFlags("battery reset service unavailable");
            publishLowBatteryMetrics("reset_service_unavailable");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        battery_reset_client_->async_send_request(
            request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                bool success = false;
                std::string response_message;
                try {
                    const auto response = future.get();
                    if (response) {
                        success = response->success;
                        response_message = response->message;
                    } else {
                        response_message = "empty response";
                    }
                } catch (const std::exception &ex) {
                    response_message = ex.what();
                }

                if (success) {
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        low_battery_charge_count_ += 1;
                        low_battery_mechanism_time_sec_ += last_low_battery_return_time_sec_;
                        low_battery_return_distance_total_m_ += last_low_battery_return_distance_m_;
                        last_low_battery_event_ = "recharged";
                    }
                    logEnergyStateTransition(
                        "charging",
                        "explore_ready",
                        "battery reset complete, robot can accept new exploration task");
                    RCLCPP_INFO(this->get_logger(), "充电完成，恢复探索: %s", response_message.c_str());
                    publishTaskEvent("recharge_completed", response_message);
                    resetLowBatteryFlags("battery recharged");
                    publishLowBatteryMetrics("recharged");
                    publishZeroVelocity();
                    return;
                }

                RCLCPP_ERROR(this->get_logger(), "充电失败: %s", response_message.c_str());
                publishTaskEvent("recharge_failed", response_message);
                resetLowBatteryFlags("battery reset failed");
                publishLowBatteryMetrics("reset_failed");
            });
    }

    bool batteryResetReady()
    {
        if (battery_reset_client_ && battery_reset_client_->service_is_ready()) {
            return true;
        }
        return battery_reset_client_ &&
               battery_reset_client_->wait_for_service(std::chrono::milliseconds(500));
    }

    bool batteryDischargeControlReady()
    {
        if (battery_discharge_client_ && battery_discharge_client_->service_is_ready()) {
            return true;
        }
        return battery_discharge_client_ &&
               battery_discharge_client_->wait_for_service(std::chrono::milliseconds(500));
    }

    void setBatteryDischargeEnabled(bool enabled, const std::string &reason)
    {
        if (!battery_discharge_client_) {
            return;
        }

        if (!batteryDischargeControlReady()) {
            RCLCPP_WARN(
                this->get_logger(),
                "电池放电控制服务不可用: service=%s reason=%s",
                battery_discharge_service_.c_str(),
                reason.c_str());
            return;
        }

        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = enabled;
        battery_discharge_client_->async_send_request(
            request,
            [this, enabled, reason](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                try {
                    const auto response = future.get();
                    if (!response || !response->success) {
                        RCLCPP_WARN(
                            this->get_logger(),
                            "电池放电%s失败: reason=%s response=%s",
                            enabled ? "恢复" : "暂停",
                            reason.c_str(),
                            response ? response->message.c_str() : "empty response");
                        return;
                    }

                    RCLCPP_INFO(
                        this->get_logger(),
                        "电池放电%s: reason=%s",
                        enabled ? "恢复" : "暂停",
                        reason.c_str());
                } catch (const std::exception &ex) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "电池放电%s服务异常: reason=%s error=%s",
                        enabled ? "恢复" : "暂停",
                        reason.c_str(),
                        ex.what());
                }
            });
    }

    void resetLowBatteryFlags(const std::string &reason)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            low_battery_handling_active_ = false;
            returning_home_active_ = false;
            charging_active_ = false;
            pending_return_home_plan_ = false;
            return_home_retry_count_ = 0;
            nav_goal_active_ = false;
            current_goal_handle_.reset();
            RCLCPP_INFO(this->get_logger(), "低电量流程结束: %s", reason.c_str());
        }
        setBatteryDischargeEnabled(true, reason);
    }

    bool scheduleReturnHomeRetry(const std::string &reason)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            low_battery_handling_active_ = true;
            returning_home_active_ = true;
            charging_active_ = false;
            pending_return_home_plan_ = false;
        }

        clearTaskState(reason);
        publishZeroVelocity();
        publishTaskEvent("return_home_retry_scheduled", reason);
        publishLowBatteryMetrics("return_home_retry_scheduled");

        if (!startReturnHomeTask()) {
            publishTaskEvent("return_home_retry_dispatch_failed", reason);
            publishLowBatteryMetrics("return_home_retry_dispatch_failed");
            publishZeroVelocity();
            return false;
        }
        return true;
    }

    void publishZeroVelocity()
    {
        if (!cmd_vel_pub_) {
            return;
        }
        geometry_msgs::msg::Twist zero_twist;
        cmd_vel_pub_->publish(zero_twist);
    }

    void publishLowBatteryMetrics(const std::string &event_name)
    {
        if (!low_battery_metrics_pub_) {
            return;
        }

        double total_time_sec = 0.0;
        double total_distance_m = 0.0;
        int charge_count = 0;
        double goal_settle_wait_time_sec = 0.0;
        double last_distance_m = 0.0;
        double last_time_sec = 0.0;
        double last_percentage = 1.0;
        bool active = false;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            total_time_sec = low_battery_mechanism_time_sec_;
            total_distance_m = low_battery_return_distance_total_m_;
            charge_count = low_battery_charge_count_;
            goal_settle_wait_time_sec = goal_settle_wait_time_sec_;
            last_distance_m = last_low_battery_return_distance_m_;
            last_time_sec = last_low_battery_return_time_sec_;
            last_percentage = last_low_battery_percentage_;
            last_low_battery_event_ = event_name;
            active = low_battery_handling_active_;
        }

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        oss << "{"
            << "\"event\":\"" << event_name << "\","
            << "\"namespace\":\"" << this->get_namespace() << "\","
            << "\"timestamp_unix\":" << static_cast<double>(std::time(nullptr)) << ","
            << "\"low_battery_threshold\":" << low_battery_threshold_ << ","
            << "\"last_battery_percentage\":" << last_percentage << ","
            << "\"charge_count\":" << charge_count << ","
            << "\"low_battery_mechanism_time_sec\":" << total_time_sec << ","
            << "\"goal_settle_wait_time_sec\":" << goal_settle_wait_time_sec << ","
            << "\"return_distance_total_m\":" << total_distance_m << ","
            << "\"last_estimated_return_distance_m\":" << last_distance_m << ","
            << "\"last_estimated_return_time_sec\":" << last_time_sec << ","
            << "\"low_battery_handling_active\":" << (active ? "true" : "false") << ","
            << "\"home_x\":" << home_x_ << ","
            << "\"home_y\":" << home_y_
            << "}";

        std_msgs::msg::String msg;
        msg.data = oss.str();
        low_battery_metrics_pub_->publish(msg);
    }

    double estimateReturnDistanceMeters(
        double robot_x,
        double robot_y,
        double goal_x,
        double goal_y) const
    {
        nav_msgs::msg::OccupancyGrid::SharedPtr gmap;
        {
            std::lock_guard<std::mutex> lock(global_map_mutex_);
            gmap = global_map_;
        }

        const double euclidean_distance = std::hypot(goal_x - robot_x, goal_y - robot_y);
        if (!gmap) {
            return euclidean_distance;
        }

        const auto path = planPathOnGlobalMap(*gmap, robot_x, robot_y, goal_x, goal_y);
        if (path.size() < 2) {
            return euclidean_distance;
        }

        return computePathLengthMeters(path);
    }

    static double computePathLengthMeters(const std::vector<geometry_msgs::msg::Point> &path)
    {
        double total_distance = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            total_distance += pointDistance(path[i - 1], path[i]);
        }
        return total_distance;
    }

    void publishPlanningFailureFrontier(int frontier_id, const std::string &reason)
    {
        if (frontier_id < 0 || !planning_failure_pub_) {
            return;
        }

        std_msgs::msg::Int32 msg;
        msg.data = frontier_id;
        planning_failure_pub_->publish(msg);
        RCLCPP_WARN(
            this->get_logger(),
            "上报规划失败 frontier_id=%d, reason=%s，建议目标分配器临时切换其它候选",
            frontier_id,
            reason.c_str());
    }

    bool planAndDispatch(bool is_replan, const std::string &reason)
    {
        nav_msgs::msg::OccupancyGrid::SharedPtr gmap;
        {
            std::lock_guard<std::mutex> lock(global_map_mutex_);
            gmap = global_map_;
        }
        if (!gmap) {
            RCLCPP_WARN(this->get_logger(), "无法规划: 尚未收到 global_map");
            return false;
        }

        double robot_x = 0.0;
        double robot_y = 0.0;
        if (!getRobotPoseInFrame(global_frame_, robot_x, robot_y)) {
            RCLCPP_WARN(this->get_logger(), "无法规划: 获取机器人在 %s 中的位姿失败", global_frame_.c_str());
            return false;
        }

        geometry_msgs::msg::Point raw_goal;
        int frontier_id = -1;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!task_active_) {
                return false;
            }
            raw_goal = final_goal_global_;
            frontier_id = current_frontier_id_;
        }

        int goal_gx = 0;
        int goal_gy = 0;
        if (!worldToGrid(*gmap, raw_goal.x, raw_goal.y, goal_gx, goal_gy)) {
            RCLCPP_WARN(this->get_logger(), "目标点不在 global_map 内，无法规划");
            publishPlanningFailureFrontier(frontier_id, "goal outside global_map");
            return false;
        }

        int traversable_goal_gx = goal_gx;
        int traversable_goal_gy = goal_gy;
        if (!findNearestTraversableCell(*gmap, goal_gx, goal_gy, traversable_goal_gx, traversable_goal_gy)) {
            RCLCPP_WARN(this->get_logger(), "目标点附近未找到可通行栅格，无法规划");
            publishPlanningFailureFrontier(frontier_id, "no traversable cell near goal");
            return false;
        }

        geometry_msgs::msg::Point adjusted_goal = gridToWorld(*gmap, traversable_goal_gx, traversable_goal_gy);
        adjusted_goal = applyObstacleRepulsion(*gmap, adjusted_goal);

        auto planned_path = planPathOnGlobalMap(*gmap, robot_x, robot_y, adjusted_goal.x, adjusted_goal.y);
        if (planned_path.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "A* 规划失败或路径过短");
            publishPlanningFailureFrontier(frontier_id, "astar failed or path too short");
            return false;
        }

        if (planned_path.size() > 2) {
            for (size_t i = 1; i + 1 < planned_path.size(); ++i) {
                planned_path[i] = applyObstacleRepulsion(*gmap, planned_path[i]);
            }
        }

        const size_t next_index = selectNextWaypointIndex(planned_path, 0);

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!task_active_) {
                return false;
            }
            adjusted_goal_global_ = adjusted_goal;
            global_path_ = planned_path;
            current_waypoint_index_ = next_index;
        }

        publishVisualization();

        RCLCPP_INFO(this->get_logger(),
                    "%s任务 %d: A*路径点数=%zu, next_index=%zu, adjusted_goal=(%.2f, %.2f), reason=%s",
                    is_replan ? "重规划" : "初始规划", frontier_id, planned_path.size(), next_index,
                    adjusted_goal.x, adjusted_goal.y, reason.c_str());

        return dispatchWaypoint(next_index, is_replan ? "replan" : "plan");
    }

    bool replanCurrentTask(const std::string &reason)
    {
        int frontier_id = -1;
        int retries = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!task_active_) {
                return false;
            }
            if (!enable_explore_replanning_) {
                frontier_id = current_frontier_id_;
                RCLCPP_WARN(
                    this->get_logger(),
                    "任务 %d 的自动重规划已关闭，忽略重规划请求: %s",
                    frontier_id,
                    reason.c_str());
                return false;
            }
            if (replan_retry_count_ >= max_replan_retries_) {
                frontier_id = current_frontier_id_;
                retries = replan_retry_count_;
                RCLCPP_WARN(this->get_logger(),
                            "任务 %d 已达到最大重规划次数 %d，停止当前任务",
                            frontier_id, retries);
                return false;
            }
            ++replan_retry_count_;
            frontier_id = current_frontier_id_;
            retries = replan_retry_count_;
            nav_goal_active_ = false;
            current_goal_handle_.reset();
            best_remaining_distance_ = std::numeric_limits<double>::infinity();
            last_progress_time_ = this->now();
            last_feedback_time_ = this->now();
        }

        RCLCPP_WARN(this->get_logger(), "任务 %d 开始第 %d 次重规划, reason=%s",
                    frontier_id, retries, reason.c_str());
        publishTaskEvent("task_replan_requested", reason);
        return planAndDispatch(true, reason);
    }

    bool dispatchWaypoint(size_t path_index, const std::string &reason)
    {
        geometry_msgs::msg::Point subgoal_global;
        int frontier_id = -1;
        double score = 0.0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!task_active_ || path_index >= global_path_.size()) {
                return false;
            }
            subgoal_global = global_path_[path_index];
            current_subgoal_global_ = subgoal_global;
            current_waypoint_index_ = path_index;
            frontier_id = current_frontier_id_;
            score = current_score_;
            best_remaining_distance_ = std::numeric_limits<double>::infinity();
            last_progress_time_ = this->now();
            last_feedback_time_ = this->now();
            cancel_requested_for_replan_ = false;
        }

        double nav_x = 0.0;
        double nav_y = 0.0;
        if (!transformPointBetweenFrames(subgoal_global.x, subgoal_global.y,
                                         global_frame_, target_frame_, nav_x, nav_y)) {
            RCLCPP_WARN(this->get_logger(), "子目标 TF 变换失败，尝试重规划");
            return replanCurrentTask("subgoal tf transform failed");
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = target_frame_;
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = nav_x;
        goal_msg.pose.pose.position.y = nav_y;
        goal_msg.pose.pose.position.z = 0.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options;

        send_goal_options.goal_response_callback =
            [this, frontier_id, path_index](GoalHandleNavigate::SharedPtr goal_handle) {
                bool rejected_return_home = false;
                bool ignore_response = false;
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    if (!task_active_ || frontier_id != current_frontier_id_) {
                        ignore_response = true;
                    } else if (!goal_handle) {
                        nav_goal_active_ = false;
                        current_goal_handle_.reset();
                        rejected_return_home = returning_home_active_;
                        if (rejected_return_home) {
                            task_active_ = false;
                            global_path_.clear();
                            current_waypoint_index_ = 0;
                        }
                    } else {
                        current_goal_handle_ = goal_handle;
                        nav_goal_active_ = true;
                    }
                }

                if (ignore_response) {
                    return;
                }

                if (rejected_return_home) {
                    RCLCPP_ERROR(this->get_logger(), "返航子目标被 Nav2 拒绝");
                    publishTaskEvent("return_home_goal_rejected", "return-home subgoal rejected by Nav2");
                    publishVisualization();
                    publishLowBatteryMetrics("return_home_goal_rejected");
                    scheduleReturnHomeRetry("return-home goal rejected");
                    return;
                }
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "任务 %d 的子目标 %zu 被 Nav2 拒绝",
                                 frontier_id, path_index);
                    publishTaskEvent("task_goal_rejected", "subgoal rejected by Nav2");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "任务 %d 的子目标 %zu 已被 Nav2 接受",
                            frontier_id, path_index);
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleNavigate::SharedPtr,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (!task_active_ || !nav_goal_active_) {
                    return;
                }
                last_feedback_time_ = this->now();
                if (feedback && std::isfinite(feedback->distance_remaining)) {
                    if (feedback->distance_remaining + min_progress_delta_m_ < best_remaining_distance_) {
                        best_remaining_distance_ = feedback->distance_remaining;
                        last_progress_time_ = this->now();
                    }
                }
            };

        send_goal_options.result_callback =
            [this](const GoalHandleNavigate::WrappedResult &result) {
                this->handleNavResult(result);
            };

        nav2_client_->async_send_goal(goal_msg, send_goal_options);

        publishVisualization();

        RCLCPP_INFO(this->get_logger(),
                    "发送任务 %d 的子目标 %zu: global=(%.2f, %.2f) -> %s=(%.2f, %.2f), score=%.3f, reason=%s",
                    frontier_id, path_index,
                    subgoal_global.x, subgoal_global.y,
                    target_frame_.c_str(), nav_x, nav_y,
                    score, reason.c_str());
        return true;
    }

    void handleNavResult(const GoalHandleNavigate::WrappedResult &result)
    {
        bool should_start_return_home = false;
        bool should_request_recharge = false;
        bool need_replan = false;
        bool need_send_next_waypoint = false;
        bool should_finish = false;
        bool continuing_return_home = false;
        size_t next_waypoint_index = 0;
        int frontier_id = -1;
        std::string finish_reason;
        std::string replan_reason;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!task_active_) {
                return;
            }

            frontier_id = current_frontier_id_;
            nav_goal_active_ = false;
            current_goal_handle_.reset();
            best_remaining_distance_ = std::numeric_limits<double>::infinity();

            if (pending_return_home_plan_) {
                pending_return_home_plan_ = false;
                should_start_return_home = true;
            } else if (returning_home_active_ && result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                const bool reached_last_waypoint = global_path_.empty() ||
                                                   current_waypoint_index_ >= global_path_.size() - 1;
                const double final_dist = pointDistance(current_subgoal_global_, adjusted_goal_global_);

                if (reached_last_waypoint || final_dist <= final_goal_tolerance_m_) {
                    returning_home_active_ = false;
                    charging_active_ = true;
                    logEnergyStateTransition(
                        "return_home",
                        "charging",
                        "home reached, request battery reset");
                    should_request_recharge = true;
                    should_finish = false;
                } else {
                    next_waypoint_index = selectNextWaypointIndex(global_path_, current_waypoint_index_);
                    if (next_waypoint_index <= current_waypoint_index_) {
                        next_waypoint_index = std::min(current_waypoint_index_ + 1, global_path_.size() - 1);
                    }
                    need_send_next_waypoint = true;
                    continuing_return_home = true;
                }
            } else if (returning_home_active_ &&
                       (result.code == rclcpp_action::ResultCode::CANCELED ||
                        result.code == rclcpp_action::ResultCode::ABORTED)) {
                should_start_return_home = true;
                finish_reason = (result.code == rclcpp_action::ResultCode::CANCELED)
                                    ? "return-home canceled"
                                    : "return-home aborted";
            } else if (returning_home_active_) {
                should_start_return_home = true;
                finish_reason = "return-home failed";
            } else
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                const bool reached_last_waypoint = global_path_.empty() ||
                                                   current_waypoint_index_ >= global_path_.size() - 1;
                const double final_dist = pointDistance(current_subgoal_global_, adjusted_goal_global_);

                if (reached_last_waypoint || final_dist <= final_goal_tolerance_m_) {
                    should_finish = true;
                    finish_reason = "final waypoint reached";
                } else {
                    next_waypoint_index = selectNextWaypointIndex(global_path_, current_waypoint_index_);
                    if (next_waypoint_index <= current_waypoint_index_) {
                        next_waypoint_index = std::min(current_waypoint_index_ + 1, global_path_.size() - 1);
                    }
                    need_send_next_waypoint = true;
                }
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                if (cancel_requested_for_frontier_disappearance_) {
                    cancel_requested_for_frontier_disappearance_ = false;
                    should_finish = true;
                    finish_reason = "target frontier disappeared";
                } else if (cancel_requested_for_replan_) {
                    cancel_requested_for_replan_ = false;
                    if (enable_explore_replanning_) {
                        need_replan = true;
                        replan_reason = "goal canceled for replanning";
                    } else {
                        should_finish = true;
                        finish_reason = "goal canceled while explore replanning disabled";
                    }
                } else {
                    should_finish = true;
                    finish_reason = "goal canceled externally";
                }
            } else {
                if (enable_explore_replanning_) {
                    need_replan = true;
                    replan_reason = (result.code == rclcpp_action::ResultCode::ABORTED)
                                        ? "goal aborted"
                                        : "unknown nav result";
                } else {
                    should_finish = true;
                    finish_reason = (result.code == rclcpp_action::ResultCode::ABORTED)
                                        ? "goal aborted while explore replanning disabled"
                                        : "nav result without replanning";
                }
            }
        }

        if (should_start_return_home) {
            publishTaskEvent("return_home_failed", finish_reason);
            scheduleReturnHomeRetry(finish_reason);
            return;
        }

        if (should_request_recharge) {
            publishTaskEvent("return_home_reached", "home reached, request battery reset");
            publishZeroVelocity();
            clearTaskState("return-home reached");
            requestBatteryRecharge();
            return;
        }

        if (should_finish) {
            if (finish_reason == "final waypoint reached" && enable_goal_settle_wait_ &&
                goal_settle_wait_sec_ > 1e-6) {
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    nav_goal_active_ = false;
                    current_goal_handle_.reset();
                    goal_settle_wait_active_ = true;
                    goal_settle_wait_start_ = this->now();
                    goal_settle_wait_deadline_ =
                        this->now() + rclcpp::Duration::from_seconds(goal_settle_wait_sec_);
                }
                publishZeroVelocity();
                setBatteryDischargeEnabled(false, "goal settle wait started");
                RCLCPP_INFO(
                    this->get_logger(),
                    "任务 %d 到达终点，静置等待 %.1f 秒后再释放任务",
                    frontier_id, goal_settle_wait_sec_);
                publishTaskEvent("task_settle_wait_started", "final waypoint reached, waiting for map update");
                publishLowBatteryMetrics("goal_settle_wait_started");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "任务 %d 完成: %s", frontier_id, finish_reason.c_str());
            publishTaskEvent(
                finish_reason == "final waypoint reached" ? "task_completed" : "task_finished",
                finish_reason);
            clearTaskState(finish_reason);
            return;
        }

        if (need_send_next_waypoint) {
            if (!dispatchWaypoint(next_waypoint_index, "waypoint succeeded")) {
                if (continuing_return_home) {
                    publishTaskEvent("return_home_failed", "next return-home waypoint dispatch failed");
                    scheduleReturnHomeRetry("next return-home waypoint dispatch failed");
                } else {
                    if (!replanCurrentTask("next waypoint dispatch failed")) {
                        clearTaskState("next waypoint dispatch failed");
                    }
                }
            }
            return;
        }

        if (need_replan) {
            if (!replanCurrentTask(replan_reason)) {
                clearTaskState(replan_reason);
            }
        }
    }

    bool getRobotPoseInFrame(const std::string &frame, double &x, double &y) const
    {
        try {
            const auto transform = tf_buffer_->lookupTransform(
                frame, base_frame_, tf2::TimePointZero,
                tf2::durationFromSec(tf_timeout_sec_));
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "获取机器人位姿失败 %s <- %s: %s",
                        frame.c_str(), base_frame_.c_str(), ex.what());
            return false;
        }
    }

    bool transformPointBetweenFrames(double in_x, double in_y,
                                     const std::string &source_frame,
                                     const std::string &target_frame,
                                     double &out_x, double &out_y) const
    {
        if (source_frame == target_frame) {
            out_x = in_x;
            out_y = in_y;
            return true;
        }

        geometry_msgs::msg::PointStamped src;
        src.header.stamp = this->now();
        src.header.frame_id = source_frame;
        src.point.x = in_x;
        src.point.y = in_y;
        src.point.z = 0.0;

        try {
            const auto transform = tf_buffer_->lookupTransform(
                target_frame, source_frame, tf2::TimePointZero,
                tf2::durationFromSec(tf_timeout_sec_));
            geometry_msgs::msg::PointStamped dst;
            tf2::doTransform(src, dst, transform);
            out_x = dst.point.x;
            out_y = dst.point.y;
            return true;
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "点变换失败 %s -> %s: %s",
                        source_frame.c_str(), target_frame.c_str(), ex.what());
            return false;
        }
    }

    static bool worldToGrid(const nav_msgs::msg::OccupancyGrid &map,
                            double wx, double wy,
                            int &gx, int &gy)
    {
        const double origin_x = map.info.origin.position.x;
        const double origin_y = map.info.origin.position.y;
        const double resolution = map.info.resolution;
        gx = static_cast<int>(std::floor((wx - origin_x) / resolution));
        gy = static_cast<int>(std::floor((wy - origin_y) / resolution));
        return gx >= 0 && gy >= 0 &&
               gx < static_cast<int>(map.info.width) &&
               gy < static_cast<int>(map.info.height);
    }

    static geometry_msgs::msg::Point gridToWorld(const nav_msgs::msg::OccupancyGrid &map,
                                                 int gx, int gy)
    {
        geometry_msgs::msg::Point p;
        p.x = map.info.origin.position.x + (static_cast<double>(gx) + 0.5) * map.info.resolution;
        p.y = map.info.origin.position.y + (static_cast<double>(gy) + 0.5) * map.info.resolution;
        p.z = 0.0;
        return p;
    }

    geometry_msgs::msg::Point applyObstacleRepulsion(const nav_msgs::msg::OccupancyGrid &map,
                                                     const geometry_msgs::msg::Point &input_point) const
    {
        if (repulsion_max_shift_m_ <= 0.0) {
            return input_point;
        }

        int center_gx = 0;
        int center_gy = 0;
        if (!worldToGrid(map, input_point.x, input_point.y, center_gx, center_gy)) {
            return input_point;
        }

        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        const int radius = std::max(1, repulsion_window_radius_cells_);
        const double resolution = map.info.resolution;
        const double max_window_distance = std::hypot(
            (static_cast<double>(radius) + 0.5) * resolution,
            (static_cast<double>(radius) + 0.5) * resolution);

        double repel_x = 0.0;
        double repel_y = 0.0;
        double min_obstacle_distance = std::numeric_limits<double>::infinity();
        int obstacle_count = 0;

        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                const int gx = center_gx + dx;
                const int gy = center_gy + dy;
                if (gx < 0 || gy < 0 || gx >= width || gy >= height) {
                    continue;
                }

                const int idx = gy * width + gx;
                const int8_t value = map.data[idx];
                if (value < 0 || value < goal_obstacle_threshold_) {
                    continue;
                }

                const auto obstacle_point = gridToWorld(map, gx, gy);
                double vx = input_point.x - obstacle_point.x;
                double vy = input_point.y - obstacle_point.y;
                double distance = std::hypot(vx, vy);

                if (distance < 1e-6) {
                    vx = static_cast<double>(center_gx - gx);
                    vy = static_cast<double>(center_gy - gy);
                    distance = std::hypot(vx, vy) * resolution;
                    if (distance < 1e-6) {
                        continue;
                    }
                    const double norm = std::hypot(vx, vy);
                    vx /= norm;
                    vy /= norm;
                } else {
                    vx /= distance;
                    vy /= distance;
                }

                const double weight = std::max(0.0, (max_window_distance - distance) / max_window_distance);
                if (weight <= 0.0) {
                    continue;
                }

                repel_x += vx * weight;
                repel_y += vy * weight;
                min_obstacle_distance = std::min(min_obstacle_distance, distance);
                ++obstacle_count;
            }
        }

        if (obstacle_count == 0) {
            return input_point;
        }

        const double repel_norm = std::hypot(repel_x, repel_y);
        if (repel_norm < 1e-6) {
            return input_point;
        }

        const double shift_scale = std::clamp(
            (max_window_distance - min_obstacle_distance) / std::max(max_window_distance, 1e-6),
            0.0, 1.0);
        const double shift_distance = repulsion_max_shift_m_ * shift_scale;
        if (shift_distance < 1e-4) {
            return input_point;
        }

        geometry_msgs::msg::Point output_point = input_point;
        output_point.x += (repel_x / repel_norm) * shift_distance;
        output_point.y += (repel_y / repel_norm) * shift_distance;
        output_point.z = input_point.z;

        int out_gx = 0;
        int out_gy = 0;
        if (!worldToGrid(map, output_point.x, output_point.y, out_gx, out_gy)) {
            return input_point;
        }

        return output_point;
    }

    bool isCellBlockedForGoal(const nav_msgs::msg::OccupancyGrid &map, int gx, int gy) const
    {
        if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.info.width) || gy >= static_cast<int>(map.info.height)) {
            return true;
        }
        const int idx = gy * static_cast<int>(map.info.width) + gx;
        const int8_t v = map.data[idx];
        return (v < 0) || (v >= goal_obstacle_threshold_);
    }

    bool isCellTraversableForAstar(const nav_msgs::msg::OccupancyGrid &map, int gx, int gy) const
    {
        if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.info.width) || gy >= static_cast<int>(map.info.height)) {
            return false;
        }
        const int idx = gy * static_cast<int>(map.info.width) + gx;
        const int8_t v = map.data[idx];
        return (v >= 0) && (v < goal_obstacle_threshold_);
    }

    bool findNearestTraversableCell(const nav_msgs::msg::OccupancyGrid &map,
                                    int start_gx, int start_gy,
                                    int &out_gx, int &out_gy) const
    {
        if (isCellTraversableForAstar(map, start_gx, start_gy)) {
            out_gx = start_gx;
            out_gy = start_gy;
            return true;
        }

        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        const int max_radius = std::max(width, height);

        for (int r = 1; r <= max_radius; ++r) {
            for (int dx = -r; dx <= r; ++dx) {
                for (int dy = -r; dy <= r; ++dy) {
                    if (std::abs(dx) != r && std::abs(dy) != r) {
                        continue;
                    }
                    const int gx = start_gx + dx;
                    const int gy = start_gy + dy;
                    if (isCellTraversableForAstar(map, gx, gy)) {
                        out_gx = gx;
                        out_gy = gy;
                        return true;
                    }
                }
            }
        }
        return false;
    }

    std::vector<geometry_msgs::msg::Point> planPathOnGlobalMap(const nav_msgs::msg::OccupancyGrid &map,
                                                               double start_x, double start_y,
                                                               double goal_x, double goal_y) const
    {
        std::vector<geometry_msgs::msg::Point> sparse_path;

        int start_gx = 0;
        int start_gy = 0;
        int goal_gx = 0;
        int goal_gy = 0;
        if (!worldToGrid(map, start_x, start_y, start_gx, start_gy) ||
            !worldToGrid(map, goal_x, goal_y, goal_gx, goal_gy)) {
            return sparse_path;
        }
        if (!findNearestTraversableCell(map, start_gx, start_gy, start_gx, start_gy) ||
            !findNearestTraversableCell(map, goal_gx, goal_gy, goal_gx, goal_gy)) {
            return sparse_path;
        }

        const int width = static_cast<int>(map.info.width);
        const int height = static_cast<int>(map.info.height);
        const int total = width * height;
        const int start_idx = start_gy * width + start_gx;
        const int goal_idx = goal_gy * width + goal_gx;
        const double resolution = map.info.resolution;

        if (start_idx == goal_idx) {
            sparse_path.push_back(gridToWorld(map, start_gx, start_gy));
            sparse_path.push_back(gridToWorld(map, goal_gx, goal_gy));
            return sparse_path;
        }

        struct QueueNode {
            double f_cost;
            double g_cost;
            int idx;
            bool operator<(const QueueNode &other) const { return f_cost > other.f_cost; }
        };

        auto heuristic = [resolution](int x0, int y0, int x1, int y1) {
            return resolution * std::hypot(static_cast<double>(x0 - x1), static_cast<double>(y0 - y1));
        };

        static const std::array<std::pair<int, int>, 8> kDirs = {{
            {1, 0}, {-1, 0}, {0, 1}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        }};

        std::priority_queue<QueueNode> open_set;
        std::vector<double> g_score(total, std::numeric_limits<double>::infinity());
        std::vector<uint8_t> closed(total, 0);
        std::vector<int> parent(total, -1);

        g_score[start_idx] = 0.0;
        open_set.push({heuristic(start_gx, start_gy, goal_gx, goal_gy), 0.0, start_idx});

        bool found = false;
        int expansions = 0;

        while (!open_set.empty()) {
            const auto current = open_set.top();
            open_set.pop();

            if (closed[current.idx]) {
                continue;
            }
            closed[current.idx] = 1;

            if (current.idx == goal_idx) {
                found = true;
                break;
            }

            if (++expansions > astar_max_expansions_) {
                RCLCPP_WARN(this->get_logger(), "A* 达到最大扩展数 %d，终止搜索", astar_max_expansions_);
                break;
            }

            const int cx = current.idx % width;
            const int cy = current.idx / width;

            for (const auto &dir : kDirs) {
                const int nx = cx + dir.first;
                const int ny = cy + dir.second;
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
                    continue;
                }

                const int nidx = ny * width + nx;
                if (closed[nidx]) {
                    continue;
                }

                const int8_t cell_value = map.data[nidx];
                if (cell_value >= 0 && cell_value >= goal_obstacle_threshold_) {
                    continue;
                }

                const bool diagonal = (dir.first != 0 && dir.second != 0);
                if (diagonal) {
                    const int side_idx_1 = cy * width + (cx + dir.first);
                    const int side_idx_2 = (cy + dir.second) * width + cx;
                    if ((map.data[side_idx_1] >= 0 && map.data[side_idx_1] >= goal_obstacle_threshold_) ||
                        (map.data[side_idx_2] >= 0 && map.data[side_idx_2] >= goal_obstacle_threshold_)) {
                        continue;
                    }
                }

                double step_cost = resolution * (diagonal ? std::sqrt(2.0) : 1.0);
                if (cell_value < 0) {
                    step_cost *= astar_unknown_cost_multiplier_;
                }

                const double tentative_g = current.g_cost + step_cost;
                if (tentative_g >= g_score[nidx]) {
                    continue;
                }

                g_score[nidx] = tentative_g;
                parent[nidx] = current.idx;
                const double f_cost = tentative_g + heuristic(nx, ny, goal_gx, goal_gy);
                open_set.push({f_cost, tentative_g, nidx});
            }
        }

        if (!found) {
            return sparse_path;
        }

        std::vector<int> idx_path;
        for (int idx = goal_idx; idx != -1; idx = parent[idx]) {
            idx_path.push_back(idx);
        }
        std::reverse(idx_path.begin(), idx_path.end());

        std::vector<geometry_msgs::msg::Point> dense_path;
        dense_path.reserve(idx_path.size());
        for (const int idx : idx_path) {
            dense_path.push_back(gridToWorld(map, idx % width, idx / width));
        }

        sparse_path.reserve(dense_path.size());
        sparse_path.push_back(dense_path.front());
        double accumulated = 0.0;
        for (size_t i = 1; i < dense_path.size(); ++i) {
            accumulated += pointDistance(dense_path[i - 1], dense_path[i]);
            if (accumulated >= subgoal_spacing_m_ || i == dense_path.size() - 1) {
                sparse_path.push_back(dense_path[i]);
                accumulated = 0.0;
            }
        }

        if (sparse_path.back().x != dense_path.back().x || sparse_path.back().y != dense_path.back().y) {
            sparse_path.push_back(dense_path.back());
        }

        return sparse_path;
    }

    bool isLocalCellKnownAndFree(const nav_msgs::msg::OccupancyGrid &map,
                                 int gx, int gy,
                                 int safety_cells) const
    {
        if (gx < safety_cells || gy < safety_cells ||
            gx >= static_cast<int>(map.info.width) - safety_cells ||
            gy >= static_cast<int>(map.info.height) - safety_cells) {
            return false;
        }

        for (int dx = -safety_cells; dx <= safety_cells; ++dx) {
            for (int dy = -safety_cells; dy <= safety_cells; ++dy) {
                const int nx = gx + dx;
                const int ny = gy + dy;
                const int idx = ny * static_cast<int>(map.info.width) + nx;
                const int8_t v = map.data[idx];
                if (v < 0 || v >= local_known_threshold_) {
                    return false;
                }
            }
        }
        return true;
    }

    bool hasLineOfSightInLocalMap(const nav_msgs::msg::OccupancyGrid &map,
                                  double x0, double y0,
                                  double x1, double y1) const
    {
        const double resolution = map.info.resolution;
        const int steps = std::max(1, static_cast<int>(std::ceil(std::hypot(x1 - x0, y1 - y0) / resolution)));
        const int safety_cells = std::max(0, static_cast<int>(std::ceil(local_safety_margin_m_ / resolution)));

        for (int i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(steps);
            const double x = x0 + t * (x1 - x0);
            const double y = y0 + t * (y1 - y0);
            int gx = 0;
            int gy = 0;
            if (!worldToGrid(map, x, y, gx, gy)) {
                return false;
            }
            if (!isLocalCellKnownAndFree(map, gx, gy, safety_cells)) {
                return false;
            }
        }
        return true;
    }

    size_t selectNextWaypointIndex(const std::vector<geometry_msgs::msg::Point> &path,
                                   size_t current_index)
    {
        if (path.empty()) {
            return 0;
        }
        if (current_index >= path.size() - 1) {
            return path.size() - 1;
        }

        nav_msgs::msg::OccupancyGrid::SharedPtr lmap;
        {
            std::lock_guard<std::mutex> lock(local_map_mutex_);
            lmap = local_map_;
        }

        double robot_local_x = 0.0;
        double robot_local_y = 0.0;
        const bool have_robot_local_pose = getRobotPoseInFrame(local_map_frame_, robot_local_x, robot_local_y);

        if (!lmap || !have_robot_local_pose) {
            double accumulated = 0.0;
            for (size_t i = current_index + 1; i < path.size(); ++i) {
                accumulated += pointDistance(path[i - 1], path[i]);
                if (accumulated >= fallback_subgoal_distance_m_ || i == path.size() - 1) {
                    return i;
                }
            }
            return path.size() - 1;
        }

        const int safety_cells = std::max(0, static_cast<int>(std::ceil(local_safety_margin_m_ / lmap->info.resolution)));
        size_t farthest_index = current_index;

        for (size_t i = current_index + 1; i < path.size(); ++i) {
            double local_x = 0.0;
            double local_y = 0.0;
            if (!transformPointBetweenFrames(path[i].x, path[i].y,
                                             global_frame_, local_map_frame_, local_x, local_y)) {
                break;
            }

            int gx = 0;
            int gy = 0;
            if (!worldToGrid(*lmap, local_x, local_y, gx, gy)) {
                break;
            }
            if (!isLocalCellKnownAndFree(*lmap, gx, gy, safety_cells)) {
                break;
            }
            if (!hasLineOfSightInLocalMap(*lmap, robot_local_x, robot_local_y, local_x, local_y)) {
                break;
            }
            farthest_index = i;
        }

        if (farthest_index == current_index) {
            double accumulated = 0.0;
            for (size_t i = current_index + 1; i < path.size(); ++i) {
                accumulated += pointDistance(path[i - 1], path[i]);
                if (accumulated >= fallback_subgoal_distance_m_ || i == path.size() - 1) {
                    return i;
                }
            }
            return path.size() - 1;
        }

        return farthest_index;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPointNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
