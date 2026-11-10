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
    default_poses_path = os.path.join(sim_pkg_share, "config", "robot_initial_poses_real.yaml")
    home_pose_path_arg = DeclareLaunchArgument("home_pose_path", default_value=default_poses_path)
    astar_unknown_cost_multiplier_arg = DeclareLaunchArgument("astar_unknown_cost_multiplier", default_value="100.0")
    astar_max_expansions_arg = DeclareLaunchArgument("astar_max_expansions", default_value="150000")

    def build_nodes(context):
        robot_ns = context.launch_configurations["robot_name"]
        pose_path = context.perform_substitution(LaunchConfiguration("home_pose_path"))
        with open(pose_path, "r", encoding="utf-8") as f:
            robot_poses = yaml.safe_load(f)["robot_poses"]
        home_pose = robot_poses[robot_ns]

        return [
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
                        "enable_low_battery_handling": True,
                        "discharge_during_return": False,
                        "low_battery_check_threshold": 0.25,
                        "low_battery_threshold": 0.25,
                        "astar_unknown_cost_multiplier": LaunchConfiguration("astar_unknown_cost_multiplier"),
                        "astar_max_expansions": LaunchConfiguration("astar_max_expansions"),
                        "home_x": float(home_pose["x"]),
                        "home_y": float(home_pose["y"]),
                        "battery_reset_service": "reset_battery",
                        "battery_discharge_service": "set_discharge_enabled",
                        "charge_command_topic": "charge_command",
                        "low_battery_metrics_topic": "low_battery_metrics",
                    }
                ],
                remappings=[("/task_point", "task_point")],
            )
        ]

    return LaunchDescription(
        [
            robot_name_arg,
            robot_id_arg,
            home_pose_path_arg,
            astar_unknown_cost_multiplier_arg,
            astar_max_expansions_arg,
            OpaqueFunction(function=build_nodes),
        ]
    )
