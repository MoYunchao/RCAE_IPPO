#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_sim = get_package_share_directory("simulation_bringup")

    # -----------------------------
    # Launch arguments
    # -----------------------------
    robot_count = LaunchConfiguration("robot_count")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_robot_count = DeclareLaunchArgument(
        "robot_count", default_value="3"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    # -----------------------------
    # World (关键：明确 world 路径)
    # -----------------------------
    world_file = os.path.join(
        pkg_sim,
        "world",
        "model",
        "model_semantic.world"
    )

    # -----------------------------
    # Gazebo model path
    # -----------------------------
    gazebo_model_path = os.path.join(pkg_sim, "world", "room2")

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=gazebo_model_path + ":" + os.environ.get("GAZEBO_MODEL_PATH", "")
    )

    # -----------------------------
    # Gazebo server (先启动)
    # -----------------------------
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzserver.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file
        }.items(),
    )

    # Gazebo client
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gzclient.launch.py",
            )
        )
    )

    # -----------------------------
    # 修改点1：使用OpaqueFunction动态创建机器人节点
    # -----------------------------
    def create_robot_nodes(context):
        # 从launch配置中获取robot_count的值
        count = int(context.launch_configurations['robot_count'])
        
        # Robot initial poses
        with open(
            os.path.join(pkg_sim, "config", "robot_initial_poses.yaml"), "r"
        ) as f:
            robot_poses = yaml.safe_load(f)["robot_poses"]

        robot_nodes = []

        # 修改点2：使用count而不是len(robot_poses)
        for i in range(count):
            robot_name = f"robot_{i}"
            # 使用YAML中的位置，假设YAML中有足够的位置配置
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
                    "-entity", robot_name,
                    "-topic", f"/{robot_name}/robot_description",
                    "-robot_namespace", robot_name,
                    "-x", str(pose["x"]),
                    "-y", str(pose["y"]),
                    "-z", str(pose["z"]),
                    "-Y", str(pose["yaw"]),
                ],
            )

            robot_nodes.append(robot_description)
            robot_nodes.append(spawn_robot)
            
        return robot_nodes
    
    def simulation_pose_publisher_launch(context):

        robot_count_value = int(context.launch_configurations['robot_count'])
        
        simulation_pose_publisher_node = Node(
            package="simulation_bringup",
            executable="simulation_pose_publisher.py",
            name="simulation_pose_publisher",
            output="screen",
            parameters=[
                {"publish_initial_poses": True},
                {"robot_count": robot_count_value},
            ]
        )
    
        return [simulation_pose_publisher_node]

    # -----------------------------
    # 延迟 spawn（关键稳定点）
    # -----------------------------
    delayed_spawn = TimerAction(
        period=1.0,   # 等 gzserver 完全 ready
        actions=[OpaqueFunction(function=create_robot_nodes)]
    )

    # -----------------------------
    # LaunchDescription
    # -----------------------------
    ld = LaunchDescription()

    ld.add_action(declare_robot_count)
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(delayed_spawn)
    ld.add_action(OpaqueFunction(function=simulation_pose_publisher_launch))

    return ld