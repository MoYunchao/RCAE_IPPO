from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('fuzzy_decision'),
        'rviz2',
        'nav2.rviz'
    ])
    
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'camera_frame',
            default_value='oakd_rgb_camera_frame',
            description='Camera frame name'
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='base_link',
            description='Target frame for pointcloud transformation'
        ),
        DeclareLaunchArgument(
            'robot_frame',
            default_value='base_link',
            description='Robot base frame'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame name'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/oakd/rgb/preview/image_raw',
            description='RGB camera topic'
        ),
        DeclareLaunchArgument(
            'grid_topic',
            default_value='/local_grid',
            description='Local grid topic'
        ),
        
        # 1. 包含 rgbd_to_map 包的launch文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rgbd_to_map'),
                    'launch',
                    'rgbd_to_map.launch.py'
                ])
            ]),
            launch_arguments={
                'camera_frame': LaunchConfiguration('camera_frame'),
                'target_frame': LaunchConfiguration('target_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
            }.items()
        ),
        
        # 2. 包含 clip_semantic 包的launch文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('clip_semantic'),
                    'launch',
                    'clip_semantic_launch.py'
                ])
            ]),
            launch_arguments={
                'camera_topic': LaunchConfiguration('camera_topic'),
                'grid_topic': LaunchConfiguration('grid_topic'),
            }.items()
        ),
        
        # 3. 启动 fuzzy_decision_node 节点
        Node(
            package='fuzzy_decision',
            executable='fuzzy_decision_node',
            name='fuzzy_decision_node',
            output='screen',
            parameters=[],
            remappings=[
                ('/frontiers_info', '/frontiers_info'),
                ('/frontier_priority_fuzzy', '/frontier_priority_fuzzy')
            ]
        ),
        
        Node(
            package='goal_select',
            executable='goal_selector_node',
            name='goal_select_node',
            output='screen',
        ),
        
        Node(
        package='rviz2',
        executable='rviz2',
        name='fuzzy_decision_rviz2',
        arguments=['-d', rviz_config_path],  # -d 参数指定配置文件
        output='screen',
        emulate_tty=True  # 更好的彩色输出
        ),
    ])