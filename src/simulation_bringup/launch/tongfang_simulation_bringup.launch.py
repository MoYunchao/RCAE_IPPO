#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory("simulation_bringup")

    robot_count = LaunchConfiguration("robot_count")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_robot_count = DeclareLaunchArgument("robot_count", default_value="3")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")

    world_file = os.path.join(pkg_sim, "world", "tongfang", "tongfang.world")
    poses_file = os.path.join(pkg_sim, "config", "robot_initial_poses_tongfang.yaml")

    gazebo_model_path = os.path.join(pkg_sim, "world", "room2")
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=gazebo_model_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", ""),
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py",
            )
        ),
        launch_arguments={"world": world_file}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            )
        )
    )

    def create_robot_nodes(context):
        count = int(context.launch_configurations["robot_count"])

        with open(poses_file, "r", encoding="utf-8") as f:
            robot_poses = yaml.safe_load(f)["robot_poses"]

        robot_nodes = []
        for i in range(count):
            robot_name = f"robot_{i}"
            pose = robot_poses[robot_name]

            robot_description = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=robot_name,
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "frame_prefix": f"{robot_name}/",
                        "robot_description": os.popen(
                            f"xacro {pkg_sim}/urdf/sensebeetle_original.xacro "
                            f"robot_namespace:={robot_name}"
                        ).read(),
                    }
                ],
            )

            spawn_robot = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=[
                    "-entity",
                    robot_name,
                    "-topic",
                    f"/{robot_name}/robot_description",
                    "-robot_namespace",
                    robot_name,
                    "-x",
                    str(pose["x"]),
                    "-y",
                    str(pose["y"]),
                    "-z",
                    str(pose["z"]),
                    "-Y",
                    str(pose["yaw"]),
                ],
            )

            robot_nodes.append(robot_description)
            robot_nodes.append(spawn_robot)

        return robot_nodes

    def simulation_pose_publisher_launch(context):
        robot_count_value = int(context.launch_configurations["robot_count"])
        simulation_pose_publisher_node = Node(
            package="simulation_bringup",
            executable="simulation_pose_publisher.py",
            name="simulation_pose_publisher",
            output="screen",
            parameters=[
                {"publish_initial_poses": True},
                {"robot_count": robot_count_value},
                {"poses_config_file": poses_file},
            ],
        )
        return [simulation_pose_publisher_node]

    delayed_spawn = TimerAction(
        period=1.0,
        actions=[OpaqueFunction(function=create_robot_nodes)],
    )

    ld = LaunchDescription()
    ld.add_action(declare_robot_count)
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(delayed_spawn)
    ld.add_action(OpaqueFunction(function=simulation_pose_publisher_launch))
    return ld
