import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.actions import TimerAction


def launch_setup(context, *args, **kwargs):
    # ========== 1. 取真实字符串 ==========
    robot_name = context.launch_configurations['robot_name']
    use_sim_time = context.launch_configurations['use_sim_time'].lower() == 'true'

    # ========== 2. 拼接所有需要的字符串 ==========
    map_frame   = f'{robot_name}/map'
    odom_frame  = f'{robot_name}/odom'
    base_frame  = f'{robot_name}/base_link'
    livox_parent_frame = f'{robot_name}/{robot_name}_livox_frame'
    livox_child_frame  = f'{robot_name}_livox_frame'

    # ========== 3. 配置文件路径 ==========
    robot_nav_pkg_share = get_package_share_directory('robot_navigation')
    resource_monitor_pkg_share = get_package_share_directory('resource_monitor')
    slam_params_file = os.path.join(
        robot_nav_pkg_share, 'params', 'mapper_params_online_async.yaml'
    )
    resource_monitor_config = os.path.join(
        resource_monitor_pkg_share, 'config', 'resource_monitor_config.yaml'
    )
    nav_to_pose_bt_xml = os.path.join(
        robot_nav_pkg_share,
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_follow_gap.xml'
    )
    nav_through_poses_bt_xml = os.path.join(
        robot_nav_pkg_share,
        'behavior_trees',
        'navigate_w_replanning_and_follow_gap.xml'
    )
    nav2_params = os.path.join(
        robot_nav_pkg_share, 'params', 'nav2_params_mppi_robot_1.yaml'
    )
    
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        root_key=robot_name,
        param_rewrites={
            'use_sim_time': str(use_sim_time).lower(),
            'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
        },
        convert_types=True
    )
    
    # ========== 4. static TF ==========
    start_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='livox_tf_fix',
        namespace=robot_name,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', livox_parent_frame,
            '--child-frame-id', livox_child_frame
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ========== 5. pointcloud_to_laserscan ==========
    start_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': livox_child_frame,
            'transform_tolerance': 0.1,
            'min_height': 0.05,
            'max_height': 0.3,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0030,
            'scan_time': 0.1667,
            'range_min': 0.05,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        remappings=[
            ('cloud_in', 'livox/lidar/pointcloud'),
            ('scan', 'scan')
        ]
    )

    # ========== 6. odom -> TF ==========
    start_odom_to_tf_node = Node(
        package='robot_navigation',
        executable='odom_to_tf.py',
        name='odom_to_tf_broadcaster',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': 'odom',
            'odom_frame': odom_frame,
            'base_frame': base_frame,
        }]
    )

    resource_monitor_node = Node(
        package='resource_monitor',
        executable='resource_monitor',
        name='resource_monitor',
        namespace=robot_name,
        output='screen',
        parameters=[
            resource_monitor_config,
            {
                'mode': 'simulation',
                'robot_id': 1,
                'robot_namespace': 'robot_',
            }
        ],
        emulate_tty=True,
    )

    # ========== 7. SLAM Toolbox ==========
    start_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace=robot_name,
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': use_sim_time,
                'map_frame': map_frame,
                'use_msg_time': True,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'scan_topic': 'scan',
            }
        ],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'odom'),
            ('/map', '/robot_1/map')
        ]
    )
    
    map_postprocessor_node = Node(
        package='map_postprocessor',
        executable='map_postprocessor_node',
        namespace='robot_1',
        name='map_postprocessor',
        output='screen',
        parameters=[{
            # TF frame 还是要显式写
            'map_frame': 'robot_1/map',
            'base_frame': 'robot_1/base_link',

            # 改成相对话题名，让 namespace 自动生效
            'input_topic': 'map',
            'output_topic': 'map_postprocessed',

            'free_threshold': 25,
            'unknown_value': -1,
            'use_robot_position': True,
            'min_process_interval': 0.1,
            'dilation_radius': 2,
            'ray_angle_step': 0.5,
        }],
    )
    
    remappings = []
    
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn =True,
        respawn_delay=2.0,
        namespace=robot_name,
        parameters=[configured_nav2_params],
        remappings=remappings,
    )
    
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        respawn =True,
        respawn_delay=2.0,
        namespace=robot_name, 
        parameters=[configured_nav2_params],
        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
    )
    
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=robot_name,
        output='screen',
        respawn =True,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        remappings=remappings,
    )
    
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=robot_name,
        output='screen',
        respawn =True,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        remappings=remappings,
    )
    
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace=robot_name,
        output='screen',
        respawn =True,
        respawn_delay=2.0,
        parameters=[configured_nav2_params],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel')
        ],
    )
    
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        namespace=robot_name,
        respawn =False,
        respawn_delay=5.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother'
            ],
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True
        }]
    )
    
    delayed_start_slam_toolbox = TimerAction(
        period=4.0,
        actions=[start_slam_toolbox_node,map_postprocessor_node]
    )
    
    nav2_nodes_group=[
        controller_server,
        planner_server,
        bt_navigator,
        behavior_server,
        velocity_smoother,
        lifecycle_manager
    ]
    
    delayed_lifecycle_manager = TimerAction(
        period=8.0,
        actions=nav2_nodes_group
    )

    return [
        start_static_tf_node,
        start_pointcloud_to_laserscan_node,
        start_odom_to_tf_node,
        resource_monitor_node,
        delayed_start_slam_toolbox,
        delayed_lifecycle_manager
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/robot_1/camera/image_raw',
            description='RGB图像话题'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot_1',
            description='Robot namespace / prefix'
        ),
        DeclareLaunchArgument(
            'battery_design_capacity_ah',
            default_value='0.50',
            description='Battery design capacity in Ah'
        ),
        OpaqueFunction(function=launch_setup),
        Node(
            package='battery_sim',
            executable='battery_state_sim',
            name='battery_state_sim',
            output='screen',
            namespace= 'robot_1',
            parameters=[{
                'update_rate_hz': 2.0,  # Hz
                'design_capacity_ah': LaunchConfiguration('battery_design_capacity_ah'),
                'initial_charge_ratio': 1.0,
                'reset_charge_ratio': 1.0,

                'camera_current': 0.06,  # A
                'lidar_current': 0.06,  # A
                'compute_current_gain': 2.0,
                'communication_current_gain': 1.0,
                'max_bandwidth_mbps': 10.0,
                'failure_charge_ratio': 0.0,
                'idle_current': 0.04,  # A
                'kv': 8.0,
                'kw': 1.2
            }]
        ),

        Node(
            package='battery_sim',
            executable='battery_manager',
            name='battery_manager',
            output='screen',
            namespace='robot_1',
            parameters=[{
                'auto_recover':False,
                'nav2_nodes':[
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother'
                ]
            }]
        )
    ])
