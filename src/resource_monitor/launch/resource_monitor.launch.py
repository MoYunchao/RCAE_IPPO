#!/usr/bin/env python3
"""
Resource Monitor Launch File

启动资源监控节点
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 参数声明
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='simulation',
        description='兼容参数（当前节点固定 simulation 模式）'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='机器人 ID'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot_',
        description='机器人命名空间前缀'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('resource_monitor'),
            'config',
            'resource_monitor_config.yaml'
        ]),
        description='配置文件路径'
    )
    
    # 资源监控节点
    resource_monitor_node = Node(
        package='resource_monitor',
        executable='resource_monitor',
        name='resource_monitor',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'mode': LaunchConfiguration('mode'),
                'robot_id': LaunchConfiguration('robot_id'),
                'robot_namespace': LaunchConfiguration('robot_namespace'),
            }
        ],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        mode_arg,
        robot_id_arg,
        robot_namespace_arg,
        config_file_arg,
        resource_monitor_node,
    ])
