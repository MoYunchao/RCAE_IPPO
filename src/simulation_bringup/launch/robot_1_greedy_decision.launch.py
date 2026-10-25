import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ns = 'robot_1'
    robot_id = 1
    sim_pkg_share = get_package_share_directory('simulation_bringup')
    default_poses_path = os.path.join(sim_pkg_share, 'config', 'robot_initial_poses.yaml')

    def build_nodes(context):
        poses_path = context.perform_substitution(LaunchConfiguration('home_pose_path'))
        with open(poses_path, 'r', encoding='utf-8') as f:
            robot_poses = yaml.safe_load(f)['robot_poses']
        home_pose = robot_poses[robot_ns]

        return [
            Node(
                package='fuzzy_decision',
                executable='greedy_decision_node',
                name='greedy_decision_node',
                namespace=robot_ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'weight_extremely_high': 10.0,
                    'weight_low': 2.0,
                    'robot_id': robot_id,
                    'local_map_topic': f'/{robot_ns}/map',
                    'local_map_frame': f'{robot_ns}/map',
                }],
                remappings=[
                    ('frontiers_info', '/frontiers_info'),
                    ('/frontier_priority_fuzzy', 'frontier_priority_fuzzy')
                ]
            ),
            Node(
                package='multi_robot_navigation',
                executable='minimal_task_navigator',
                name='task_point_navigator',
                namespace=robot_ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'target_frame': f'{robot_ns}/map',
                    'local_map_frame': f'{robot_ns}/map',
                    'global_frame': 'global_map',
                    'base_frame': f'{robot_ns}/base_link',
                    'global_map_topic': '/global_map',
                    'local_map_topic': f'/{robot_ns}/map',
                    'enable_low_battery_handling': False,
                    'low_battery_check_threshold': 0.25,
                    'low_battery_threshold': 0.25,
                    'astar_unknown_cost_multiplier': LaunchConfiguration('astar_unknown_cost_multiplier'),
                    'astar_max_expansions': LaunchConfiguration('astar_max_expansions'),
                    'home_x': float(home_pose['x']),
                    'home_y': float(home_pose['y']),
                    'battery_reset_service': 'reset_battery',
                    'battery_discharge_service': 'set_discharge_enabled',
                    'low_battery_metrics_topic': 'low_battery_metrics',
                }],
                remappings=[
                    ('/task_point', 'task_point'),
                ],
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'home_pose_path',
            default_value=default_poses_path,
            description='Path to robot initial poses yaml used as return-home targets'
        ),
        DeclareLaunchArgument(
            'astar_unknown_cost_multiplier',
            default_value='100.0',
            description='Unknown-cell cost multiplier used by navigator A*'
        ),
        DeclareLaunchArgument(
            'astar_max_expansions',
            default_value='150000',
            description='Maximum expansions allowed in navigator A*'
        ),
        OpaqueFunction(function=build_nodes),
    ])
