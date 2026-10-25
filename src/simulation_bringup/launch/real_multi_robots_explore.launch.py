#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, LifecycleTransition, Node
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    sim_pkg = get_package_share_directory("simulation_bringup")

    robot_count_arg = DeclareLaunchArgument(
        "robot_count",
        default_value="3",
        description="Number of robots for RL training",
    )

    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value="robot_",
        description="Namespace prefix for robots",
    )

    global_frame_arg = DeclareLaunchArgument(
        "global_frame",
        default_value="global_map",
        description="The global frame_id for all operations",
    )

    robot_poses_arg = DeclareLaunchArgument(
        "robot_poses",
        default_value="",
        description="Robot initial poses as JSON: [[x,y,yaw],...] (empty = use default)",
    )

    metrics_csv_path_arg = DeclareLaunchArgument(
        "metrics_csv_path",
        default_value=os.path.join(os.getcwd(), "data", "metrics", "energy_mission_metrics.csv"),
        description="Path to run-level CSV summary file",
    )

    metrics_summary_path_arg = DeclareLaunchArgument(
        "metrics_summary_path",
        default_value=os.path.join(os.getcwd(), "data", "metrics", "energy_mission_summary.json"),
        description="Path to run-level JSON summary file",
    )
    snapshot_times_sec_arg = DeclareLaunchArgument(
        "snapshot_times_sec",
        default_value="100,200,300,400,500,600,700,800,900,1000",
        description="Comma-separated elapsed seconds for global_map snapshots",
    )

    dynamic_tf_broadcaster = LifecycleNode(
        package="cure_explore",
        executable="dynamic_tf_broadcaster",
        name="dynamic_tf_broadcaster",
        namespace="",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "robot_count": LaunchConfiguration("robot_count"),
                "global_frame": LaunchConfiguration("global_frame"),
            }
        ],
    )

    tf_broadcaster_configure = RegisterEventHandler(
        OnProcessStart(
            target_action=dynamic_tf_broadcaster,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[
                        LifecycleTransition(
                            lifecycle_node_names=["dynamic_tf_broadcaster"],
                            transition_ids=[Transition.TRANSITION_CONFIGURE],
                        )
                    ],
                )
            ],
        )
    )

    tf_broadcaster_activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dynamic_tf_broadcaster,
            goal_state="inactive",
            entities=[
                TimerAction(
                    period=0.5,
                    actions=[
                        LifecycleTransition(
                            lifecycle_node_names=["dynamic_tf_broadcaster"],
                            transition_ids=[Transition.TRANSITION_ACTIVATE],
                        )
                    ],
                )
            ],
        )
    )

    map_merger_node = Node(
        package="cure_explore_cpp",
        executable="map_merger_node",
        name="map_merger",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "robot_count": LaunchConfiguration("robot_count"),
                "robot_namespace": LaunchConfiguration("robot_namespace"),
                "global_frame": LaunchConfiguration("global_frame"),
                "update_rate": 5.0,
                "use_visibility_filter": True,
            }
        ],
    )

    global_semantic_map = Node(
        package="clip_semantic",
        executable="oracle_semantic_mapper",
        name="semantic_mapper",
        output="screen",
        parameters=[
            {
                "world_sdf_path": os.path.join(
                    get_package_share_directory("simulation_bringup"),
                    "world",
                    "real",
                    "real_semantic.world",
                ),
                "static_grid_path": os.path.join(
                    get_package_share_directory("clip_semantic"),
                    "config",
                    "static_semantic_grid_real.npz",
                ),
                "publish_period_sec": 1.0,
            }
        ],
    )

    frontier_info_node = Node(
        package="rgbd_to_map",
        executable="frontier_detector",
        name="frontier_detector",
        output="screen",
        parameters=[
            {
                "update_rate": 5.0,
                "dbscan_eps": 0.25,
                "dbscan_min_points": 3,
                "min_cluster_size": 40,
                "semantic_search_radius": 0.2,
                "gain_unknown_radius": 3.0,
                "robot_base_frames": ["robot_0/base_link", "robot_1/base_link", "robot_2/base_link"],
                "robot_names": ["robot_0", "robot_1", "robot_2"],
            }
        ],
    )

    coverage_supervisor = Node(
        package="simulation_bringup",
        executable="exploration_supervisor.py",
        name="exploration_supervisor",
        output="screen",
        parameters=[
            {
                "robot_count": LaunchConfiguration("robot_count"),
                "global_map_topic": "/global_map",
                "stop_topic": "/exploration_stop",
                "coverage_target": 0.95,
                "max_mission_time_sec": 0.0,
                "coverage_hold_sec": 3.0,
                "min_runtime_sec": 20.0,
                "check_period_sec": 1.0,
                "roi_min_x": -33.5,
                "roi_min_y": -19.5,
                "roi_max_x": 30.5,
                "roi_max_y": 19.5,
                "coverage_total_override": 182114,
                "static_map_yaml": os.path.join(sim_pkg, "world", "real", "real.yaml"),
                "static_map_count_mode": "free_only",
                "csv_path": LaunchConfiguration("metrics_csv_path"),
                "summary_path": LaunchConfiguration("metrics_summary_path"),
                "max_odom_step_m": 1.0,
            }
        ],
    )

    trajectory_snapshot_recorder = Node(
        package="simulation_bringup",
        executable="trajectory_snapshot_recorder.py",
        name="trajectory_snapshot_recorder",
        output="screen",
        parameters=[
            {
                "robot_count": LaunchConfiguration("robot_count"),
                "robot_namespace": LaunchConfiguration("robot_namespace"),
                "global_map_topic": "/global_map",
                "summary_path": LaunchConfiguration("metrics_summary_path"),
                "snapshot_times_sec": LaunchConfiguration("snapshot_times_sec"),
                "trajectory_min_step_m": 0.02,
                "trajectory_sample_period_sec": 0.5,
            }
        ],
    )

    delayed_start = TimerAction(
        period=4.0,
        actions=[map_merger_node, global_semantic_map, frontier_info_node, coverage_supervisor, trajectory_snapshot_recorder],
    )

    return LaunchDescription(
        [
            robot_count_arg,
            robot_namespace_arg,
            global_frame_arg,
            robot_poses_arg,
            metrics_csv_path_arg,
            metrics_summary_path_arg,
            snapshot_times_sec_arg,
            dynamic_tf_broadcaster,
            tf_broadcaster_configure,
            tf_broadcaster_activate,
            delayed_start,
        ]
    )
