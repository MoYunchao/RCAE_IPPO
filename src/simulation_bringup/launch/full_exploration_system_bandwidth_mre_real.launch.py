import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory("simulation_bringup")
    rl_pkg = get_package_share_directory("rl_training")
    home_pose_path = os.path.join(sim_pkg, "config", "robot_initial_poses_real.yaml")

    model_path_arg = DeclareLaunchArgument("model_path")
    metrics_csv_path_arg = DeclareLaunchArgument(
        "metrics_csv_path",
        default_value=os.path.join(os.getcwd(), "data", "metrics", "coverage95_metrics.csv"),
    )
    metrics_summary_path_arg = DeclareLaunchArgument(
        "metrics_summary_path",
        default_value=os.path.join(os.getcwd(), "data", "metrics", "coverage95_summary.json"),
    )
    infer_save_enabled_arg = DeclareLaunchArgument("infer_save_enabled", default_value="true")
    infer_save_root_arg = DeclareLaunchArgument("infer_save_root", default_value="output/rl_runs")
    infer_run_name_arg = DeclareLaunchArgument("infer_run_name", default_value="bandwidth_mre_infer")
    robot0_battery_capacity_arg = DeclareLaunchArgument("robot0_battery_capacity_ah", default_value="0.75")
    robot1_battery_capacity_arg = DeclareLaunchArgument("robot1_battery_capacity_ah", default_value="0.78")
    robot2_battery_capacity_arg = DeclareLaunchArgument("robot2_battery_capacity_ah", default_value="0.79")
    slam_delay_arg = DeclareLaunchArgument("slam_delay_sec", default_value="6.0")
    multi_explore_delay_arg = DeclareLaunchArgument("multi_explore_delay_sec", default_value="26.0")
    navigator_delay_arg = DeclareLaunchArgument("navigator_delay_sec", default_value="34.0")
    rl_delay_arg = DeclareLaunchArgument("rl_delay_sec", default_value="38.0")
    rviz_delay_arg = DeclareLaunchArgument("rviz_delay_sec", default_value="40.0")
    device_arg = DeclareLaunchArgument("device", default_value="auto")

    env_launch = os.path.join(sim_pkg, "launch", "real_simulation_bringup.launch.py")
    robot0_slam_launch = os.path.join(sim_pkg, "launch", "robot_0_slam.launch.py")
    robot1_slam_launch = os.path.join(sim_pkg, "launch", "robot_1_slam.launch.py")
    robot2_slam_launch = os.path.join(sim_pkg, "launch", "robot_2_slam.launch.py")
    multi_explore_launch = os.path.join(sim_pkg, "launch", "real_multi_robots_explore.launch.py")
    navigator_launch = os.path.join(sim_pkg, "launch", "robot_task_navigator_real.launch.py")
    rl_launch = os.path.join(rl_pkg, "launch", "rl_bandwidth_mre.launch.py")
    rviz_config = os.path.join(sim_pkg, "config", "fuzzy_decision.rviz")

    simulation_env = IncludeLaunchDescription(PythonLaunchDescriptionSource(env_launch))
    robot0_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot0_slam_launch),
        launch_arguments={
            "use_sim_time": "true",
            "robot_name": "robot_0",
            "battery_design_capacity_ah": LaunchConfiguration("robot0_battery_capacity_ah"),
        }.items(),
    )
    robot1_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot1_slam_launch),
        launch_arguments={
            "use_sim_time": "true",
            "robot_name": "robot_1",
            "battery_design_capacity_ah": LaunchConfiguration("robot1_battery_capacity_ah"),
        }.items(),
    )
    robot2_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot2_slam_launch),
        launch_arguments={
            "use_sim_time": "true",
            "robot_name": "robot_2",
            "battery_design_capacity_ah": LaunchConfiguration("robot2_battery_capacity_ah"),
        }.items(),
    )

    multi_robot_explore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(multi_explore_launch),
        launch_arguments={
            "robot_count": "3",
            "robot_namespace": "robot_",
            "home_pose_path": home_pose_path,
            "global_frame": "global_map",
            "metrics_csv_path": LaunchConfiguration("metrics_csv_path"),
            "metrics_summary_path": LaunchConfiguration("metrics_summary_path"),
        }.items(),
    )

    robot0_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigator_launch),
        launch_arguments={"robot_name": "robot_0", "robot_id": "0", "home_pose_path": home_pose_path}.items(),
    )
    robot1_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigator_launch),
        launch_arguments={"robot_name": "robot_1", "robot_id": "1", "home_pose_path": home_pose_path}.items(),
    )
    robot2_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigator_launch),
        launch_arguments={"robot_name": "robot_2", "robot_id": "2", "home_pose_path": home_pose_path}.items(),
    )

    rl_inference = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rl_launch),
        launch_arguments={
            "model_path": LaunchConfiguration("model_path"),
            "robot_count": "3",
            "robot_namespace": "robot_",
            "device": LaunchConfiguration("device"),
            "task_topic_template": "/{robot_name}/task_point",
            "debug_priority_topic_template": "/{robot_name}/bandwidth_mre_priority_debug",
            "global_map_topic": "/global_map",
            "frontier_topic": "/frontiers_info",
            "odom_topic_template": "/{robot_name}/odom",
            "rate": "2.0",
            "local_radius_m": "12.0",
            "decision_min_distance_m": "3.0",
            "decision_max_distance_m": "9.0",
            "utility_radius_m": "18.0",
            "utility_top_k": "5",
            "guidepost_top_k": "5",
            "coord_norm_scale_m": "40.0",
            "save_enabled": LaunchConfiguration("infer_save_enabled"),
            "save_root": LaunchConfiguration("infer_save_root"),
            "run_name": LaunchConfiguration("infer_run_name"),
            "task_metrics_topic_template": "/{robot_name}/task_metrics",
            "low_battery_metrics_topic_template": "/{robot_name}/low_battery_metrics",
        }.items(),
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            model_path_arg,
            metrics_csv_path_arg,
            metrics_summary_path_arg,
            infer_save_enabled_arg,
            infer_save_root_arg,
            infer_run_name_arg,
            robot0_battery_capacity_arg,
            robot1_battery_capacity_arg,
            robot2_battery_capacity_arg,
            slam_delay_arg,
            multi_explore_delay_arg,
            navigator_delay_arg,
            rl_delay_arg,
            rviz_delay_arg,
            device_arg,
            simulation_env,
            TimerAction(period=LaunchConfiguration("slam_delay_sec"), actions=[robot0_slam, robot1_slam, robot2_slam]),
            TimerAction(period=LaunchConfiguration("multi_explore_delay_sec"), actions=[multi_robot_explore]),
            TimerAction(period=LaunchConfiguration("navigator_delay_sec"), actions=[robot0_nav, robot1_nav, robot2_nav]),
            TimerAction(period=LaunchConfiguration("rl_delay_sec"), actions=[rl_inference]),
            TimerAction(period=LaunchConfiguration("rviz_delay_sec"), actions=[rviz2_node]),
        ]
    )
