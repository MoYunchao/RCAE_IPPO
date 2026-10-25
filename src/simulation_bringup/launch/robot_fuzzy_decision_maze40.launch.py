import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument("robot_name")
    robot_id_arg = DeclareLaunchArgument("robot_id")
    sim_pkg_share = get_package_share_directory("simulation_bringup")
    default_poses_path = os.path.join(sim_pkg_share, "config", "robot_initial_poses_maze40.yaml")
    home_pose_path_arg = DeclareLaunchArgument(
        "home_pose_path",
        default_value=default_poses_path,
        description="Path to robot initial poses yaml used as return-home targets",
    )
    weight_topic_arg = DeclareLaunchArgument(
        "weight_topic",
        default_value="/rl_adjust_weights",
        description="RL weight topic for this robot",
    )
    wait_for_rl_weights_arg = DeclareLaunchArgument(
        "wait_for_rl_weights",
        default_value="false",
        description="Whether fuzzy decision waits for RL weights before publishing priorities",
    )
    poses_path = default_poses_path

    def build_nodes(context):
        robot_ns = context.launch_configurations["robot_name"]
        robot_id = int(context.launch_configurations["robot_id"])
        pose_path = context.perform_substitution(LaunchConfiguration("home_pose_path"))

        with open(pose_path, "r", encoding="utf-8") as f:
            robot_poses = yaml.safe_load(f)["robot_poses"]
        home_pose = robot_poses[robot_ns]

        return [
            Node(
                package="fuzzy_decision",
                executable="fuzzy_decision_node",
                name="fuzzy_decision_node",
                namespace=robot_ns,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "weight_extremely_high": 10.0,
                        "weight_high": 8.0,
                        "weight_medium": 5.0,
                        "weight_low": 2.0,
                        "robot_id": robot_id,
                        "local_map_topic": f"/{robot_ns}/map",
                        "local_map_frame": f"{robot_ns}/map",
                        "weight_topic": LaunchConfiguration("weight_topic"),
                        "wait_for_rl_weights": LaunchConfiguration("wait_for_rl_weights"),
                    }
                ],
                remappings=[
                    ("frontiers_info", "/frontiers_info"),
                    ("/frontier_priority_fuzzy", "frontier_priority_fuzzy"),
                ],
            ),
            Node(
                package="multi_robot_navigation",
                executable="minimal_task_navigator",
                name="task_point_navigator",
                namespace=robot_ns,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "target_frame": f"{robot_ns}/map",
                        "local_map_frame": f"{robot_ns}/map",
                        "global_frame": "global_map",
                        "base_frame": f"{robot_ns}/base_link",
                        "global_map_topic": "/global_map",
                        "local_map_topic": f"/{robot_ns}/map",
                        "enable_low_battery_handling": False,
                        "low_battery_check_threshold": 0.25,
                        "low_battery_threshold": 0.25,
                        "astar_unknown_cost_multiplier": 25.0,
                        "astar_max_expansions": 250000,
                        "home_x": float(home_pose["x"]),
                        "home_y": float(home_pose["y"]),
                        "battery_reset_service": "reset_battery",
                        "battery_discharge_service": "set_discharge_enabled",
                        "low_battery_metrics_topic": "low_battery_metrics",
                        "enable_frontier_disappearance_switch": True,
                        "frontier_topic": "/frontiers_info",
                        "frontier_missing_hold_sec": 2.0,
                        "frontier_position_tolerance_m": 1.0,
                        "frontier_disappearance_ignore_near_goal_m": 3.0,
                    }
                ],
                remappings=[("/task_point", "task_point")],
            ),
        ]

    return LaunchDescription(
        [
            robot_name_arg,
            robot_id_arg,
            home_pose_path_arg,
            weight_topic_arg,
            wait_for_rl_weights_arg,
            OpaqueFunction(function=build_nodes),
        ]
    )
