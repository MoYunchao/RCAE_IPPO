#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path_arg = DeclareLaunchArgument("model_path", default_value="")
    robot_count_arg = DeclareLaunchArgument("robot_count", default_value="3")
    robot_namespace_arg = DeclareLaunchArgument("robot_namespace", default_value="robot_")
    global_map_topic_arg = DeclareLaunchArgument("global_map_topic", default_value="/global_map")
    frontier_topic_arg = DeclareLaunchArgument("frontier_topic", default_value="/frontiers_info")
    odom_topic_template_arg = DeclareLaunchArgument("odom_topic_template", default_value="/{robot_name}/odom")
    task_topic_template_arg = DeclareLaunchArgument(
        "task_topic_template",
        default_value="/{robot_name}/task_point",
    )
    debug_priority_topic_template_arg = DeclareLaunchArgument(
        "debug_priority_topic_template",
        default_value="/{robot_name}/bandwidth_mre_priority_debug",
    )
    rate_arg = DeclareLaunchArgument("rate", default_value="2.0")
    device_arg = DeclareLaunchArgument("device", default_value="auto")
    local_radius_arg = DeclareLaunchArgument("local_radius_m", default_value="12.0")
    decision_min_distance_arg = DeclareLaunchArgument("decision_min_distance_m", default_value="3.0")
    decision_max_distance_arg = DeclareLaunchArgument("decision_max_distance_m", default_value="9.0")
    sample_spacing_arg = DeclareLaunchArgument("sample_spacing_m", default_value="1.5")
    edge_radius_arg = DeclareLaunchArgument("edge_radius_m", default_value="2.5")
    utility_radius_arg = DeclareLaunchArgument("utility_radius_m", default_value="18.0")
    utility_top_k_arg = DeclareLaunchArgument("utility_top_k", default_value="5")
    guidepost_top_k_arg = DeclareLaunchArgument("guidepost_top_k", default_value="5")
    coord_norm_scale_arg = DeclareLaunchArgument("coord_norm_scale_m", default_value="40.0")
    target_hold_arg = DeclareLaunchArgument("target_hold_sec", default_value="8.0")
    goal_tol_arg = DeclareLaunchArgument("goal_reached_tolerance_m", default_value="1.0")
    startup_warmup_arg = DeclareLaunchArgument("startup_warmup_sec", default_value="8.0")
    require_all_ready_arg = DeclareLaunchArgument("require_all_robots_ready", default_value="true")
    log_interval_arg = DeclareLaunchArgument("log_interval", default_value="2.0")
    save_enabled_arg = DeclareLaunchArgument("save_enabled", default_value="true")
    save_root_arg = DeclareLaunchArgument("save_root", default_value="output/rl_runs")
    run_name_arg = DeclareLaunchArgument("run_name", default_value="bandwidth_mre_infer")
    task_metrics_topic_template_arg = DeclareLaunchArgument(
        "task_metrics_topic_template",
        default_value="/{robot_name}/task_metrics",
    )
    low_battery_metrics_topic_template_arg = DeclareLaunchArgument(
        "low_battery_metrics_topic_template",
        default_value="/{robot_name}/low_battery_metrics",
    )

    infer_node = Node(
        package="rl_training",
        executable="rl_infer_bandwidth_mre",
        name="rl_infer_bandwidth_mre",
        output="screen",
        arguments=[
            "--model",
            LaunchConfiguration("model_path"),
            "--robot-count",
            LaunchConfiguration("robot_count"),
            "--robot-namespace",
            LaunchConfiguration("robot_namespace"),
            "--global-map-topic",
            LaunchConfiguration("global_map_topic"),
            "--frontier-topic",
            LaunchConfiguration("frontier_topic"),
            "--odom-topic-template",
            LaunchConfiguration("odom_topic_template"),
            "--task-topic-template",
            LaunchConfiguration("task_topic_template"),
            "--debug-priority-topic-template",
            LaunchConfiguration("debug_priority_topic_template"),
            "--rate",
            LaunchConfiguration("rate"),
            "--device",
            LaunchConfiguration("device"),
            "--local-radius-m",
            LaunchConfiguration("local_radius_m"),
            "--decision-min-distance-m",
            LaunchConfiguration("decision_min_distance_m"),
            "--decision-max-distance-m",
            LaunchConfiguration("decision_max_distance_m"),
            "--sample-spacing-m",
            LaunchConfiguration("sample_spacing_m"),
            "--edge-radius-m",
            LaunchConfiguration("edge_radius_m"),
            "--utility-radius-m",
            LaunchConfiguration("utility_radius_m"),
            "--utility-top-k",
            LaunchConfiguration("utility_top_k"),
            "--guidepost-top-k",
            LaunchConfiguration("guidepost_top_k"),
            "--coord-norm-scale-m",
            LaunchConfiguration("coord_norm_scale_m"),
            "--target-hold-sec",
            LaunchConfiguration("target_hold_sec"),
            "--goal-reached-tolerance-m",
            LaunchConfiguration("goal_reached_tolerance_m"),
            "--startup-warmup-sec",
            LaunchConfiguration("startup_warmup_sec"),
            "--require-all-robots-ready",
            LaunchConfiguration("require_all_robots_ready"),
            "--log-interval",
            LaunchConfiguration("log_interval"),
            "--save-enabled",
            LaunchConfiguration("save_enabled"),
            "--save-root",
            LaunchConfiguration("save_root"),
            "--run-name",
            LaunchConfiguration("run_name"),
            "--task-metrics-topic-template",
            LaunchConfiguration("task_metrics_topic_template"),
            "--low-battery-metrics-topic-template",
            LaunchConfiguration("low_battery_metrics_topic_template"),
        ],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            model_path_arg,
            robot_count_arg,
            robot_namespace_arg,
            global_map_topic_arg,
            frontier_topic_arg,
            odom_topic_template_arg,
            task_topic_template_arg,
            debug_priority_topic_template_arg,
            rate_arg,
            device_arg,
            local_radius_arg,
            decision_min_distance_arg,
            decision_max_distance_arg,
            sample_spacing_arg,
            edge_radius_arg,
            utility_radius_arg,
            utility_top_k_arg,
            guidepost_top_k_arg,
            coord_norm_scale_arg,
            target_hold_arg,
            goal_tol_arg,
            startup_warmup_arg,
            require_all_ready_arg,
            log_interval_arg,
            save_enabled_arg,
            save_root_arg,
            run_name_arg,
            task_metrics_topic_template_arg,
            low_battery_metrics_topic_template_arg,
            infer_node,
        ]
    )
