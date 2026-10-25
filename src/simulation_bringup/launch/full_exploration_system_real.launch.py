import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory("simulation_bringup")

    metrics_csv_path_arg = DeclareLaunchArgument(
        "metrics_csv_path",
        default_value=os.path.join(os.getcwd(), "data", "metrics", "energy_mission_metrics.csv"),
    )
    metrics_summary_path_arg = DeclareLaunchArgument(
        "metrics_summary_path",
        default_value=os.path.join(os.getcwd(), "data", "metrics", "energy_mission_summary.json"),
    )
    robot0_battery_capacity_arg = DeclareLaunchArgument("robot0_battery_capacity_ah", default_value="0.91")
    robot1_battery_capacity_arg = DeclareLaunchArgument("robot1_battery_capacity_ah", default_value="0.88")
    robot2_battery_capacity_arg = DeclareLaunchArgument("robot2_battery_capacity_ah", default_value="0.82")

    env_launch = os.path.join(sim_pkg, "launch", "real_simulation_bringup.launch.py")
    robot0_slam_launch = os.path.join(sim_pkg, "launch", "robot_0_slam.launch.py")
    robot1_slam_launch = os.path.join(sim_pkg, "launch", "robot_1_slam.launch.py")
    robot2_slam_launch = os.path.join(sim_pkg, "launch", "robot_2_slam.launch.py")
    multi_explore_launch = os.path.join(sim_pkg, "launch", "real_multi_robots_explore.launch.py")
    robot_fuzzy_real_launch = os.path.join(sim_pkg, "launch", "robot_fuzzy_decision_real.launch.py")

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
            "global_frame": "global_map",
            "metrics_csv_path": LaunchConfiguration("metrics_csv_path"),
            "metrics_summary_path": LaunchConfiguration("metrics_summary_path"),
        }.items(),
    )

    robot0_fuzzy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_fuzzy_real_launch),
        launch_arguments={"robot_name": "robot_0", "robot_id": "0"}.items(),
    )
    robot1_fuzzy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_fuzzy_real_launch),
        launch_arguments={"robot_name": "robot_1", "robot_id": "1"}.items(),
    )
    robot2_fuzzy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_fuzzy_real_launch),
        launch_arguments={"robot_name": "robot_2", "robot_id": "2"}.items(),
    )

    goal_selector = Node(
        package="goal_select",
        executable="multi_robot_goal_selector_node",
        name="multi_robot_goal_selector_node",
        output="screen",
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
            metrics_csv_path_arg,
            metrics_summary_path_arg,
            robot0_battery_capacity_arg,
            robot1_battery_capacity_arg,
            robot2_battery_capacity_arg,
            simulation_env,
            TimerAction(period=6.0, actions=[robot0_slam, robot1_slam, robot2_slam]),
            TimerAction(period=26.0, actions=[multi_robot_explore]),
            TimerAction(period=34.0, actions=[robot0_fuzzy, robot1_fuzzy, robot2_fuzzy]),
            TimerAction(period=40.0, actions=[goal_selector, rviz2_node]),
        ]
    )
