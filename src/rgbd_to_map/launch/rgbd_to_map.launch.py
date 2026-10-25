from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from math import pi as M_PI

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'camera_frame',
            default_value='oakd_rgb_camera_frame',
            description='Camera frame name for RGBD processor'
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='base_link',
            description='Target frame for pointcloud transformation in RGBD processor'
        ),
        DeclareLaunchArgument(
            'robot_frame',
            default_value='base_link',
            description='Robot base frame for scan mapping'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Map frame name'
        ),
        
        # Node 2: PointCloud to LaserScan Node
        Node(
            package='rgbd_to_map',
            executable='camera_pointcloud_to_laserscan_node',
            name='camera_pointcloud_to_laserscan',
            output='screen',
            parameters=[{
                'input_topic': '/oakd/rgb/preview/depth/points',  # 使用相对话题名
                'output_topic': 'scan1',
                'target_frame': LaunchConfiguration('target_frame'),  # 使用相同的target_frame
                'min_height': 0.1,
                'max_height': 1.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': M_PI/540.0,
                'scan_time': 0.0333333,
                'range_min': 0.1,
                'range_max': 1000.0,
                'use_inf': True,
                'inf_epsilon': 1e-6
            }]
        ),
        
        # Node 3: Scan Mapping Node
        Node(
            package='rgbd_to_map',
            executable='scan_mapping_node',
            name='scan_to_grid_mapper',
            output='screen',
            parameters=[{
                'scan_topic': 'scan1',
                'grid_topic': 'local_grid',
                'robot_frame': LaunchConfiguration('robot_frame'),  # 使用robot_frame参数
                'map_frame': LaunchConfiguration('map_frame'),
                'grid_resolution': 0.05,
                'grid_width_min': 100,
                'grid_height_min': 100,
                'grid_width_max': 400,
                'grid_height_max': 400,
                'safety_margin': 1.0,
                'scale_factor': 1.2,
                'raytrace_step': 0.01,
                'dynamic_size': True,
                'max_range_threshold': 10.0
            }]
        ),
        
        # Node 4: Frontier Detector Node
        Node(
            package='rgbd_to_map',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'update_rate': 1.0,
                'dbscan_eps': 0.25,
                'dbscan_min_points': 3,
                'min_cluster_size': 10,
                'semantic_search_radius': 0.2,
                'distance_metric': 'grid_path'
            }],
            remappings=[           # 订阅局部地图
                ('/semantic_map', 'semantic_map'),
                ('/robot_pose', 'robot_pose')
            ]
        )
    ])
