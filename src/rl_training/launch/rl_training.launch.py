#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_home_pose_path = os.path.join(
        get_package_share_directory('simulation_bringup'),
        'config',
        'robot_initial_poses.yaml',
    )

    robot_count_arg = DeclareLaunchArgument('robot_count', default_value='3')
    robot_namespace_arg = DeclareLaunchArgument('robot_namespace', default_value='robot_')
    publish_rate_arg = DeclareLaunchArgument('publish_rate', default_value='2.0')
    summary_log_interval_arg = DeclareLaunchArgument('summary_log_interval', default_value='5.0')
    cpu_is_percent_arg = DeclareLaunchArgument('cpu_is_percent', default_value='false')
    bandwidth_max_arg = DeclareLaunchArgument('bandwidth_max', default_value='10.0')
    coverage_use_roi_arg = DeclareLaunchArgument('coverage_use_roi', default_value='true')
    coverage_roi_min_x_arg = DeclareLaunchArgument('coverage_roi_min_x', default_value='-15.0')
    coverage_roi_max_x_arg = DeclareLaunchArgument('coverage_roi_max_x', default_value='15.0')
    coverage_roi_min_y_arg = DeclareLaunchArgument('coverage_roi_min_y', default_value='-15.0')
    coverage_roi_max_y_arg = DeclareLaunchArgument('coverage_roi_max_y', default_value='15.0')
    coverage_total_override_arg = DeclareLaunchArgument('coverage_total_override', default_value='0')

    # 统一策略入口
    enable_weight_node_arg = DeclareLaunchArgument('enable_weight_node', default_value='true')
    policy_mode_arg = DeclareLaunchArgument(
        'policy_mode',
        default_value='fixed',
        description='fixed | mappo | torch',
    )
    weight_update_rate_arg = DeclareLaunchArgument('weight_update_rate', default_value='2.0')
    weight_log_interval_arg = DeclareLaunchArgument('weight_log_interval', default_value='2.0')
    model_path_arg = DeclareLaunchArgument('model_path', default_value='')
    infer_save_enabled_arg = DeclareLaunchArgument('infer_save_enabled', default_value='true')
    infer_save_root_arg = DeclareLaunchArgument('infer_save_root', default_value='output/rl_runs')
    infer_run_name_arg = DeclareLaunchArgument('infer_run_name', default_value='mappo_infer')
    fixed_weights_arg = DeclareLaunchArgument('fixed_weights', default_value='0.25,0.25,0.25,0.25')
    global_state_topic_arg = DeclareLaunchArgument('global_state_topic', default_value='/rl_training/global_state')
    frontier_topic_arg = DeclareLaunchArgument('frontier_topic', default_value='/frontiers_info')
    odom_topic_template_arg = DeclareLaunchArgument('odom_topic_template', default_value='/{robot_name}/odom')
    weight_topic_arg = DeclareLaunchArgument('weight_topic', default_value='/rl_training/weights')
    weight_topic_template_arg = DeclareLaunchArgument(
        'weight_topic_template',
        default_value='/{robot_name}/rl_adjust_weights',
    )
    task_metrics_topic_template_arg = DeclareLaunchArgument(
        'task_metrics_topic_template',
        default_value='/{robot_name}/task_metrics',
    )
    opportunity_top_k_arg = DeclareLaunchArgument('opportunity_top_k', default_value='5')
    candidate_distance_max_m_arg = DeclareLaunchArgument('candidate_distance_max_m', default_value='35.0')
    overlap_distance_threshold_m_arg = DeclareLaunchArgument('overlap_distance_threshold_m', default_value='2.0')
    battery_per_meter_arg = DeclareLaunchArgument('battery_per_meter', default_value='0.003')
    battery_return_margin_arg = DeclareLaunchArgument('battery_return_margin', default_value='0.05')
    battery_return_reserve_arg = DeclareLaunchArgument('battery_return_reserve', default_value='0.03')
    low_battery_threshold_arg = DeclareLaunchArgument('low_battery_threshold', default_value='0.20')
    home_pose_path_arg = DeclareLaunchArgument('home_pose_path', default_value=default_home_pose_path)
    action_mode_arg = DeclareLaunchArgument(
        'action_mode',
        default_value='auto',
        description='auto | dirichlet | beta'
    )
    torch_hidden_dim_arg = DeclareLaunchArgument('torch_hidden_dim', default_value='64')
    torch_device_arg = DeclareLaunchArgument('torch_device', default_value='cpu')
    torch_temperature_arg = DeclareLaunchArgument('torch_temperature', default_value='1.0')
    torch_smoothing_arg = DeclareLaunchArgument('torch_smoothing', default_value='0.3')

    global_state_node = Node(
        package='rl_training',
        executable='rl_global_state_node',
        name='rl_global_state_node',
        output='screen',
        parameters=[{
            'robot_count': LaunchConfiguration('robot_count'),
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'summary_log_interval_sec': LaunchConfiguration('summary_log_interval'),
            'cpu_is_percent': LaunchConfiguration('cpu_is_percent'),
            'bandwidth_max': LaunchConfiguration('bandwidth_max'),
            'coverage_use_roi': LaunchConfiguration('coverage_use_roi'),
            'coverage_roi_min_x': LaunchConfiguration('coverage_roi_min_x'),
            'coverage_roi_max_x': LaunchConfiguration('coverage_roi_max_x'),
            'coverage_roi_min_y': LaunchConfiguration('coverage_roi_min_y'),
            'coverage_roi_max_y': LaunchConfiguration('coverage_roi_max_y'),
            'coverage_total_override': LaunchConfiguration('coverage_total_override'),
        }],
        emulate_tty=True,
    )

    fixed_weight_node = Node(
        package='rl_training',
        executable='rl_fixed_weight_node',
        name='rl_fixed_weight_node',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('enable_weight_node'),
                "' == 'true' and '",
                LaunchConfiguration('policy_mode'),
                "' == 'fixed'",
            ])
        ),
        arguments=[
            '--weights', LaunchConfiguration('fixed_weights'),
            '--topic', LaunchConfiguration('weight_topic'),
            '--rate', LaunchConfiguration('weight_update_rate'),
            '--log-interval', LaunchConfiguration('weight_log_interval'),
        ],
        emulate_tty=True,
    )

    mappo_infer_node = Node(
        package='rl_training',
        executable='rl_infer_mappo',
        name='rl_infer_mappo',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('enable_weight_node'),
                "' == 'true' and '",
                LaunchConfiguration('policy_mode'),
                "' == 'mappo'",
            ])
        ),
        arguments=[
            '--model', LaunchConfiguration('model_path'),
            '--robot-count', LaunchConfiguration('robot_count'),
            '--robot-namespace', LaunchConfiguration('robot_namespace'),
            '--global-state-topic', LaunchConfiguration('global_state_topic'),
            '--per-robot-state-topic', '/rl_training/per_robot_state',
            '--frontier-topic', LaunchConfiguration('frontier_topic'),
            '--odom-topic-template', LaunchConfiguration('odom_topic_template'),
            '--weight-topic-template', LaunchConfiguration('weight_topic_template'),
            '--task-metrics-topic-template', LaunchConfiguration('task_metrics_topic_template'),
            '--rate', LaunchConfiguration('weight_update_rate'),
            '--log-interval', LaunchConfiguration('weight_log_interval'),
            '--save-enabled', LaunchConfiguration('infer_save_enabled'),
            '--save-root', LaunchConfiguration('infer_save_root'),
            '--run-name', LaunchConfiguration('infer_run_name'),
            '--opportunity-top-k', LaunchConfiguration('opportunity_top_k'),
            '--candidate-distance-max-m', LaunchConfiguration('candidate_distance_max_m'),
            '--overlap-distance-threshold-m', LaunchConfiguration('overlap_distance_threshold_m'),
            '--battery-per-meter', LaunchConfiguration('battery_per_meter'),
            '--battery-return-margin', LaunchConfiguration('battery_return_margin'),
            '--battery-return-reserve', LaunchConfiguration('battery_return_reserve'),
            '--low-battery-threshold', LaunchConfiguration('low_battery_threshold'),
            '--home-pose-path', LaunchConfiguration('home_pose_path'),
            '--action-mode', LaunchConfiguration('action_mode'),
        ],
        emulate_tty=True,
    )

    torch_weight_node = Node(
        package='rl_training',
        executable='rl_weight_node',
        name='rl_weight_node',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('enable_weight_node'),
                "' == 'true' and '",
                LaunchConfiguration('policy_mode'),
                "' == 'torch'",
            ])
        ),
        parameters=[{
            'global_state_topic': LaunchConfiguration('global_state_topic'),
            'weight_topic': LaunchConfiguration('weight_topic'),
            'model_path': LaunchConfiguration('model_path'),
            'hidden_dim': LaunchConfiguration('torch_hidden_dim'),
            'device': LaunchConfiguration('torch_device'),
            'temperature': LaunchConfiguration('torch_temperature'),
            'smoothing': LaunchConfiguration('torch_smoothing'),
            'update_rate': LaunchConfiguration('weight_update_rate'),
            'log_interval_sec': LaunchConfiguration('weight_log_interval'),
        }],
        emulate_tty=True,
    )

    return LaunchDescription([
        robot_count_arg,
        robot_namespace_arg,
        publish_rate_arg,
        summary_log_interval_arg,
        cpu_is_percent_arg,
        bandwidth_max_arg,
        coverage_use_roi_arg,
        coverage_roi_min_x_arg,
        coverage_roi_max_x_arg,
        coverage_roi_min_y_arg,
        coverage_roi_max_y_arg,
        coverage_total_override_arg,
        enable_weight_node_arg,
        policy_mode_arg,
        weight_update_rate_arg,
        weight_log_interval_arg,
        model_path_arg,
        infer_save_enabled_arg,
        infer_save_root_arg,
        infer_run_name_arg,
        fixed_weights_arg,
        global_state_topic_arg,
        frontier_topic_arg,
        odom_topic_template_arg,
        weight_topic_arg,
        weight_topic_template_arg,
        task_metrics_topic_template_arg,
        opportunity_top_k_arg,
        candidate_distance_max_m_arg,
        overlap_distance_threshold_m_arg,
        battery_per_meter_arg,
        battery_return_margin_arg,
        battery_return_reserve_arg,
        low_battery_threshold_arg,
        home_pose_path_arg,
        action_mode_arg,
        torch_hidden_dim_arg,
        torch_device_arg,
        torch_temperature_arg,
        torch_smoothing_arg,
        global_state_node,
        fixed_weight_node,
        mappo_infer_node,
        torch_weight_node,
    ])
