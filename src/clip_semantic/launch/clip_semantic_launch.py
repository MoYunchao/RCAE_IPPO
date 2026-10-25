from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 可选：声明启动参数
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/oakd/rgb/preview/image_raw',
            description='RGB图像话题'
        ),
        DeclareLaunchArgument(
            'grid_topic',
            default_value='/local_grid',
            description='局部栅格地图话题'
        ),
        
        # Node 1: CLIP语义识别节点
        Node(
            package='clip_semantic',
            executable='clip_semantic_node',
            name='clip_semantic_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/oakd/rgb/preview/image_raw', LaunchConfiguration('camera_topic')),
                ('clip_semantic', 'clip_semantic')
            ]
        ),
        
        # Node 2: 语义地图构建节点
        Node(
            package='clip_semantic',
            executable='semantic_mapper',
            name='semantic_mapper',
            output='screen',
            parameters=[],
            remappings=[
                ('/local_grid', LaunchConfiguration('grid_topic')),
                ('/clip_semantic', 'clip_semantic'),
                ('/semantic_map', 'semantic_map')
            ]
        )
    ])