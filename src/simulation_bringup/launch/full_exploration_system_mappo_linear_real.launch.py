import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory('simulation_bringup')
    rl_pkg = get_package_share_directory('rl_training')
    home_pose_path = os.path.join(sim_pkg, 'config', 'robot_initial_poses_real.yaml')

    mappo_model_path_arg = DeclareLaunchArgument('mappo_model_path')
    infer_save_enabled_arg = DeclareLaunchArgument('infer_save_enabled', default_value='true')
    infer_save_root_arg = DeclareLaunchArgument('infer_save_root', default_value='output/rl_runs')
    infer_run_name_arg = DeclareLaunchArgument('infer_run_name', default_value='')
    metrics_csv_path_arg = DeclareLaunchArgument(
        'metrics_csv_path',
        default_value=os.path.join(os.getcwd(), 'data', 'metrics', 'energy_mission_metrics.csv')
    )
    metrics_summary_path_arg = DeclareLaunchArgument(
        'metrics_summary_path',
        default_value=os.path.join(os.getcwd(), 'data', 'metrics', 'energy_mission_summary.json')
    )
    action_mode_arg = DeclareLaunchArgument('action_mode', default_value='auto')
    robot0_battery_capacity_arg = DeclareLaunchArgument('robot0_battery_capacity_ah', default_value='1.28')
    robot1_battery_capacity_arg = DeclareLaunchArgument('robot1_battery_capacity_ah', default_value='1.37')
    robot2_battery_capacity_arg = DeclareLaunchArgument('robot2_battery_capacity_ah', default_value='1.44')
    slam_delay_arg = DeclareLaunchArgument('slam_delay_sec', default_value='6.0')
    multi_explore_delay_arg = DeclareLaunchArgument('multi_explore_delay_sec', default_value='26.0')
    linear_delay_arg = DeclareLaunchArgument('linear_delay_sec', default_value='34.0')
    rl_delay_arg = DeclareLaunchArgument('rl_delay_sec', default_value='36.0')
    goal_delay_arg = DeclareLaunchArgument('goal_delay_sec', default_value='40.0')

    env_launch = os.path.join(sim_pkg, 'launch', 'real_simulation_bringup.launch.py')
    robot0_slam_launch = os.path.join(sim_pkg, 'launch', 'robot_0_slam.launch.py')
    robot1_slam_launch = os.path.join(sim_pkg, 'launch', 'robot_1_slam.launch.py')
    robot2_slam_launch = os.path.join(sim_pkg, 'launch', 'robot_2_slam.launch.py')
    multi_explore_launch = os.path.join(sim_pkg, 'launch', 'real_multi_robots_explore.launch.py')
    robot0_linear_launch = os.path.join(sim_pkg, 'launch', 'robot_0_linear_decision.launch.py')
    robot1_linear_launch = os.path.join(sim_pkg, 'launch', 'robot_1_linear_decision.launch.py')
    robot2_linear_launch = os.path.join(sim_pkg, 'launch', 'robot_2_linear_decision.launch.py')
    rl_launch = os.path.join(rl_pkg, 'launch', 'rl_training.launch.py')

    rviz_config = os.path.join(sim_pkg, 'config', 'fuzzy_decision.rviz')

    simulation_env = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(env_launch)
    )

    robot0_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot0_slam_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_name': 'robot_0',
            'battery_design_capacity_ah': LaunchConfiguration('robot0_battery_capacity_ah'),
        }.items()
    )
    robot1_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot1_slam_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_name': 'robot_1',
            'battery_design_capacity_ah': LaunchConfiguration('robot1_battery_capacity_ah'),
        }.items()
    )
    robot2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot2_slam_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_name': 'robot_2',
            'battery_design_capacity_ah': LaunchConfiguration('robot2_battery_capacity_ah'),
        }.items()
    )

    multi_robot_explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(multi_explore_launch),
        launch_arguments={
            'robot_count': '3',
            'robot_namespace': 'robot_',
            'global_frame': 'global_map',
            'metrics_csv_path': LaunchConfiguration('metrics_csv_path'),
            'metrics_summary_path': LaunchConfiguration('metrics_summary_path'),
        }.items()
    )

    robot0_linear = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot0_linear_launch),
        launch_arguments={
            'weight_topic': '/robot_0/rl_adjust_weights',
            'wait_for_rl_weights': 'true',
            'home_pose_path': home_pose_path,
        }.items()
    )
    robot1_linear = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot1_linear_launch),
        launch_arguments={
            'weight_topic': '/robot_1/rl_adjust_weights',
            'wait_for_rl_weights': 'true',
            'home_pose_path': home_pose_path,
        }.items()
    )
    robot2_linear = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot2_linear_launch),
        launch_arguments={
            'weight_topic': '/robot_2/rl_adjust_weights',
            'wait_for_rl_weights': 'true',
            'home_pose_path': home_pose_path,
        }.items()
    )

    rl_mappo_inference = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rl_launch),
        launch_arguments={
            'robot_count': '3',
            'robot_namespace': 'robot_',
            'enable_weight_node': 'true',
            'policy_mode': 'mappo',
            'model_path': LaunchConfiguration('mappo_model_path'),
            'infer_save_enabled': LaunchConfiguration('infer_save_enabled'),
            'infer_save_root': LaunchConfiguration('infer_save_root'),
            'infer_run_name': LaunchConfiguration('infer_run_name'),
            'action_mode': LaunchConfiguration('action_mode'),
            'home_pose_path': home_pose_path,
            'weight_topic_template': '/{robot_name}/rl_adjust_weights',
            'weight_update_rate': '2.0',
            'weight_log_interval': '2.0',
            'coverage_total_override': '182114',
        }.items()
    )

    goal_selector = Node(
        package='goal_select',
        executable='multi_robot_goal_selector_node',
        name='multi_robot_goal_selector_node',
        output='screen',
        parameters=[{
            'position_tolerance': 1.0,
        }],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        mappo_model_path_arg,
        infer_save_enabled_arg,
        infer_save_root_arg,
        infer_run_name_arg,
        metrics_csv_path_arg,
        metrics_summary_path_arg,
        action_mode_arg,
        robot0_battery_capacity_arg,
        robot1_battery_capacity_arg,
        robot2_battery_capacity_arg,
        slam_delay_arg,
        multi_explore_delay_arg,
        linear_delay_arg,
        rl_delay_arg,
        goal_delay_arg,
        simulation_env,
        TimerAction(period=LaunchConfiguration('slam_delay_sec'), actions=[robot0_slam, robot1_slam, robot2_slam]),
        TimerAction(period=LaunchConfiguration('multi_explore_delay_sec'), actions=[multi_robot_explore]),
        TimerAction(period=LaunchConfiguration('linear_delay_sec'), actions=[robot0_linear, robot1_linear, robot2_linear]),
        TimerAction(period=LaunchConfiguration('rl_delay_sec'), actions=[rl_mappo_inference]),
        TimerAction(period=LaunchConfiguration('goal_delay_sec'), actions=[goal_selector, rviz2_node]),
    ])
