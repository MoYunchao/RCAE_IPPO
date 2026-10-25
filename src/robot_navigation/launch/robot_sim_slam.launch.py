import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    SetEnvironmentVariable,
    RegisterEventHandler,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from nav2_common.launch import RewrittenYaml
def generate_launch_description():
    # --- 0. 声明参数 ---
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='0',
        description='Robot ID'
    )

    # [ 新增 ] 声明 'use_explore' 参数
    declare_use_explore_cmd = DeclareLaunchArgument(
        'use_explore',
        default_value='False',
        description='Whether to start the autonomous exploration node'
    )

    # 新增：控制 RViz 显示是否为完整界面
    declare_rviz_full_cmd = DeclareLaunchArgument(
        'rviz_full',
        default_value='True',
        description='Show full RViz UI when True; minimal map+TF+scan+nav info when False'
    )

    # 新增：控制是否启用 FASTLIO 定位
    declare_use_fastlio_cmd = DeclareLaunchArgument(
        'use_fastlio',
        default_value='False',
        description='Whether to use FASTLIO for odometry (True) or rely on /odom topic (False)'
    )

    # 新增：选择局部控制器类型 (dwb 或 mppi)
    declare_controller_type_cmd = DeclareLaunchArgument(
        'controller_type',
        default_value='mppi',
        description='Local controller type: dwb (default) or mppi'
    )

    # TCP通信参数
    declare_server_ip_cmd = DeclareLaunchArgument(
        'server_ip',
        default_value='127.0.0.1',
        description='TCP server IP address for reliable communication'
    )

    declare_server_port_cmd = DeclareLaunchArgument(
        'server_port',
        default_value='12140',
        description='TCP server port for reliable communication'
    )

    # 仿真模式下，统一使用仿真时间
    use_sim_time = True

    # --- 1. 定位关键文件和路径 ---
    robot_nav_pkg_share = get_package_share_directory('robot_navigation')
    fast_lio_pkg_share = get_package_share_directory("fast_lio")
    
    # [ 新增 ] 获取 'communication' 包的路径
    communication_pkg_share = get_package_share_directory('robot_communication')
    
    # [ 新增 ] 获取 'explore' 包的路径
    explore_pkg_share = get_package_share_directory('explore_lite')
    
    # Nav2 和 SLAM 的配置文件
    # 根据 controller_type 选择不同的配置文件
    nav2_params_file_dwb = os.path.join(
        robot_nav_pkg_share, 'params', 'nav2_params.yaml')
    nav2_params_file_mppi = os.path.join(
        robot_nav_pkg_share, 'params', 'nav2_params_mppi.yaml')
    slam_params_file = os.path.join(
        robot_nav_pkg_share, 'params', 'mapper_params_online_async.yaml')
    segmentation_params_file = os.path.join(
        robot_nav_pkg_share, 'params', 'segmentation_sim.yaml')
    
    # 自定义行为树路径
    nav_to_pose_bt_xml = os.path.join(
        robot_nav_pkg_share, 'behavior_trees', 'navigate_to_pose_w_replanning_and_follow_gap.xml')
    nav_through_poses_bt_xml = os.path.join(
        robot_nav_pkg_share, 'behavior_trees', 'navigate_w_replanning_and_follow_gap.xml')

    # Nav2 参数替换（对齐 navigation_launch.py 的写法，但固定 use_sim_time/autostart）
    param_substitutions = {
        'use_sim_time': 'true',   # RewrittenYaml 需要可迭代的替换项，使用字符串并启用 convert_types
        'autostart': 'true',
        'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
    }

    # DWB 配置
    configured_params_dwb = RewrittenYaml(
        source_file=nav2_params_file_dwb,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # MPPI 配置
    configured_params_mppi = RewrittenYaml(
        source_file=nav2_params_file_mppi,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )
    
    # tf remap（与 navigation_launch.py 一致）
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # 需要生命周期管理的节点列表
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    # 包含的启动文件
    nav_launch_file = os.path.join(
        robot_nav_pkg_share, 'launch', 'navigation_launch.py')
    rviz_launch_file = os.path.join(
        robot_nav_pkg_share, 'launch', 'rviz_launch.py')
    rviz_config_file = os.path.join(
        robot_nav_pkg_share, 'rviz', 'nav2.rviz')
    rviz_minimal_config_file = os.path.join(
        robot_nav_pkg_share, 'rviz', 'nav2_minimal.rviz')
        
    # FAST_LIO 启动文件
    fast_lio_launch_file = os.path.join(
        fast_lio_pkg_share, 'launch', 'mapping.launch.py')
        
    communication_launch_file = os.path.join(
        communication_pkg_share, 'launch', 'robot_communication.launch.py')

    explore_launch_file = os.path.join(
        explore_pkg_share, 'launch', 'explore.launch.py')

    # --- 3. 定义要启动的节点和包含 ---

    # **节点 1: FAST_LIO (LIO 里程计)**
    start_fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fast_lio_launch_file),
        condition=IfCondition(LaunchConfiguration('use_fastlio')),
        launch_arguments={
            'use_sim_time': str(use_sim_time)
        }.items()
    )

    # **节点 2: 地面分割 (过滤地面点云)**
    """ start_ground_segmentation_node = Node(
        package='linefit_ground_segmentation_ros',
        executable='ground_segmentation_node',
        name='ground_segmentation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'n_threads': 4,
            'r_min': 0.2,
            'r_max': 50.0,
            'n_bins': 120,
            'n_segments': 360,
            'max_dist_to_line': 0.05,
            'sensor_height': 0.21,
            'min_slope': -0.4,
            'max_slope': 0.4,
            'max_fit_error': 0.05,
            'long_threshold': 1.0,
            'max_long_height': 0.1,
            'max_start_height': 0.2,
            'line_search_angle': 0.4,
            'gravity_aligned_frame': '',
            'latch': False,
            'visualize': False,
            'input_topic': '/livox/lidar/pointcloud',
            'obstacle_output_topic': 'segmentation/obstacle',
            'ground_output_topic': 'segmentation/ground'
        }]
    ) """

    # **节点 3: PointCloud 到 LaserScan 转换器 (使用过滤后的障碍物点云)**
    start_pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': 0.05,
            'max_height': 0.3,
            'angle_min': -3.14159265,
            'angle_max': 3.14159265,
            'angle_increment': 0.0043,
            "scan_time": 0.1667,
            'range_min': 0.05,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            # 使用默认 RELIABLE QoS 发布，与所有订阅者兼容
        }],
        remappings=[
            ('cloud_in', '/livox/lidar/pointcloud'),
            ('scan_out', '/scan')
        ]
    )

    # **节点 4: SLAM (slam_toolbox)**
    # 根据 use_fastlio 参数决定 SLAM 的 odom_frame 配置
    # 如果使用 FASTLIO，SLAM 使用默认配置（FASTLIO 提供 odom->base_link）
    # 如果不使用 FASTLIO，SLAM 订阅 /odom 话题获取 odom->base_link
    slam_params_with_fastlio = [
        slam_params_file,
        {'use_sim_time': use_sim_time}
    ]
    
    slam_params_without_fastlio = [
        slam_params_file,
        {
            'use_sim_time': use_sim_time,
            'tf_buffer_duration': 3.0,  # 减少 TF 缓冲时间到 3 秒（加快重置）
        }
    ]
    
    # 启用 FASTLIO 时的 SLAM 节点
    # 地图输出重映射到 /map，由 map_postprocessor 处理后发布到 /map
    start_slam_toolbox_with_fastlio = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(LaunchConfiguration('use_fastlio')),
        parameters=slam_params_with_fastlio,
        remappings=[
            ('scan', '/scan'),
            ('map', '/map')  # 原始地图输出到 /map_raw
        ]
    )

    # 不启用 FASTLIO 时的 SLAM 节点（订阅 /odom 话题）
    start_slam_toolbox_without_fastlio = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        condition=UnlessCondition(LaunchConfiguration('use_fastlio')),
        parameters=slam_params_without_fastlio,
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),  # 显式订阅 /odom 话题
            ('map', '/map')  # 原始地图输出到 /map
        ]
    )

    # **节点: 地图后处理器 (移除不可达的孤立空闲栅格)**
    # 使用连通域分析从机器人位置进行洪水填充，清除穿墙噪声
    start_map_postprocessor_node = Node(
        package='map_postprocessor',
        executable='map_postprocessor_node',
        name='map_postprocessor',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_frame': 'map',
            'base_frame': 'base_link',
            'input_topic': '/map',
            'output_topic': '/map_postprocessed',
            'free_threshold': 25,       # 0-25 视为空闲
            'unknown_value': -1,        # 孤立区域设为未知
            'use_robot_position': True, # 从机器人位置开始洪水填充
            'min_process_interval': 0.1 # 最小处理间隔 100ms
        }]
    )

    # **节点: Odometry to TF Broadcaster (仅在不使用 FASTLIO 时启动)**
    # 当 FASTLIO 不运行时，需要这个节点从 /odom 话题发布 odom->base_link TF
    # 支持重置：订阅 reset_command 来暂停/恢复 TF 发布
    start_odom_to_tf_node = Node(
        package='robot_navigation',
        executable='odom_to_tf.py',
        name='odom_to_tf_broadcaster',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_fastlio')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'robot_id': LaunchConfiguration('robot_id')  # 用于订阅重置命令
        }]
    )

    # **Nav2 栈（仅导航，不负责定位）— 直接在此文件内启动各个节点**
    # 所有节点设置 respawn=True，重置时被杀死后自动重启
    # lifecycle_manager 也会重启并重新配置所有节点

    # 辅助函数：生成带有指定配置和条件的 Nav2 节点
    def create_nav2_nodes(configured_params, condition):
        return [
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                condition=condition,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                condition=condition,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=remappings
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                condition=condition,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=remappings
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                condition=condition,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=remappings
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                condition=condition,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=remappings
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                condition=condition,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=remappings
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=True,
                respawn_delay=2.0,
                condition=condition,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info'],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
            ),
        ]

    # 根据 controller_type 创建条件
    use_dwb_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration('controller_type'), "' == 'dwb'"])
    )
    use_mppi_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration('controller_type'), "' == 'mppi'"])
    )

    # 创建两套节点（只有满足条件的会启动）
    nav2_nodes_dwb = create_nav2_nodes(configured_params_dwb, use_dwb_condition)
    nav2_nodes_mppi = create_nav2_nodes(configured_params_mppi, use_mppi_condition)

    # lifecycle_manager 不需要条件，总是启动
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': lifecycle_nodes},
            {'bond_timeout': 10.0},
            {'attempt_respawn_reconnection': True}
        ]
    )

    # 合并所有 Nav2 节点
    nav2_nodes = nav2_nodes_dwb + nav2_nodes_mppi + [lifecycle_manager_node]

    # RViz: 全量或精简配置二选一
    start_rviz_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file),
        condition=IfCondition(LaunchConfiguration('rviz_full')),
        launch_arguments={
            'rviz_config': rviz_config_file,
            'use_namespace': 'false',
            'rviz_log_level': 'warn'
        }.items()
    )

    start_rviz_minimal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file),
        condition=UnlessCondition(LaunchConfiguration('rviz_full')),
        launch_arguments={
            'rviz_config': rviz_minimal_config_file,
            'use_namespace': 'false',
            'rviz_log_level': 'warn'
        }.items()
    )

    start_communication_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(communication_launch_file),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            # 传递 robot_id 给 communication 节点
            'robot_id': LaunchConfiguration('robot_id') 
        }.items()
    )
    
    start_explore_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(explore_launch_file),
        condition=IfCondition(LaunchConfiguration('use_explore')),
        launch_arguments={
            'use_sim_time': str(use_sim_time)
        }.items()
    )

    # **节点: TCP可靠通信客户端**
    start_tcp_client_node = Node(
        package='reliable_communication_client',
        executable='reliable_client_node',
        name='reliable_communication_client',
        output='screen',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),
            'server_ip': LaunchConfiguration('server_ip'),
            'server_port': LaunchConfiguration('server_port')
        }]
    )

    # **节点: 资源监控 (监控电池和计算资源状态)**
    start_resource_monitor_node = Node(
        package='resource_monitor',
        executable='resource_monitor_node',
        name='resource_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_id': LaunchConfiguration('robot_id'),
            'low_battery_threshold': 20.0,
            'max_tasks': 10,
            'publish_rate_hz': 1.0
        }]
    )

    # **节点: 初始旋转 (启动后自动旋转90度帮助SLAM建图)**
    start_initial_spin_node = Node(
        package='robot_navigation',
        executable='initial_spin.py',
        name='initial_spin_node',
        output='screen',
        parameters=[{
            'robot_id': LaunchConfiguration('robot_id'),  # 使用LaunchConfiguration
            'rotation_angle': 1.57,       # 90度旋转
            'rotation_speed': 0.5,        # 旋转速度 rad/s
            'startup_delay': 3.0,         # Nav2启动后3秒执行
            'linear_speed': 0.2,          # 线速度 m/s
            'move_distance': 0.5,         # 旋转后移动0.5米
            'safe_cone_angle': 0.785,     # 45度安全锥角
            'min_obstacle_distance': 1.0, # 最小1米障碍物间隙
            'max_move_attempts': 10       # 地图为空时最多移动10次
        }]
    )

    # **节点: 机器人生命周期管理器 (处理系统重置命令)**
    start_robot_lifecycle_manager = Node(
        package='robot_navigation',
        executable='robot_lifecycle_manager.py',
        name='robot_lifecycle_manager',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_id': LaunchConfiguration('robot_id'),
            'use_fastlio': LaunchConfiguration('use_fastlio'),
        }]
    )

    # --- 4. 创建并返回 LaunchDescription ---
    ld = LaunchDescription()
    
    # 设置日志缓冲环境变量（与 navigation_launch 对齐，便于日志刷新）
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    ld.add_action(stdout_linebuf_envvar)

    # 添加所有启动参数声明（仅保留必要项）
    ld.add_action(robot_id_arg)
    ld.add_action(declare_use_explore_cmd)
    ld.add_action(declare_rviz_full_cmd)
    ld.add_action(declare_use_fastlio_cmd)
    ld.add_action(declare_controller_type_cmd)
    ld.add_action(declare_server_ip_cmd)
    ld.add_action(declare_server_port_cmd)

    # ============================================================
    # 阶段 1: 启动数据源 (FAST_LIO - 提供 odom->base_link TF 和点云)
    # 同时启动资源监控节点和生命周期管理器
    # ============================================================
    ld.add_action(LogInfo(msg="[Stage 1] Starting FAST_LIO for odometry and point cloud..."))
    ld.add_action(start_fast_lio_node)
    ld.add_action(LogInfo(msg="[Stage 1] Starting resource monitor..."))
    ld.add_action(start_resource_monitor_node)
    ld.add_action(LogInfo(msg="[Stage 1] Starting robot lifecycle manager..."))
    ld.add_action(start_robot_lifecycle_manager)
    
    # ============================================================
    # 阶段 2: 等待 3 秒后启动地面分割节点 (等待 FAST_LIO 点云发布)
    # ============================================================
    """     delayed_ground_segmentation = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg="[Stage 2] Starting ground segmentation for point cloud filtering..."),
            start_ground_segmentation_node
        ]
    )
    ld.add_action(delayed_ground_segmentation) """

    # ============================================================
    # 阶段 3: 等待 2 秒后启动点云转换器 (等待地面分割输出)
    # ============================================================
    delayed_pointcloud_to_laserscan = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg="[Stage 3] Starting PointCloud to LaserScan converter..."),
            start_pointcloud_to_laserscan_node
        ]
    )
    ld.add_action(delayed_pointcloud_to_laserscan)

    # ============================================================
    # 阶段 4: 等待 4 秒后启动 SLAM (等待扫描数据和稳定的 TF)
    # 如果不使用 FASTLIO，先启动 odom_to_tf 节点发布 odom->base_link TF
    # 同时启动地图后处理器，处理 SLAM 输出的原始地图
    # ============================================================
    delayed_slam = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="[Stage 4] Starting SLAM Toolbox for mapping..."),
            start_odom_to_tf_node,  # 在 SLAM 之前启动（仅在 use_fastlio=False 时）
            start_slam_toolbox_with_fastlio,
            start_slam_toolbox_without_fastlio,
            LogInfo(msg="[Stage 4] Starting map postprocessor for noise removal..."),
            start_map_postprocessor_node  # 启动地图后处理器
        ]
    )
    ld.add_action(delayed_slam)

    # ============================================================
    # 阶段 5: 等待 8 秒后启动 Nav2 栈 (等待地图初始化和 map->odom TF)
    # 关键：给 SLAM 充足时间初始化并发布第一个 map->odom 变换
    # ============================================================
    delayed_nav2 = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[Stage 5] Starting Nav2 navigation stack..."),
        ] + nav2_nodes
    )
    ld.add_action(delayed_nav2)

    # ============================================================
    # 阶段 5.5: 等待 11 秒后启动初始旋转 (Nav2启动3秒后执行)
    # ============================================================
    delayed_initial_spin = TimerAction(
        period=11.0,
        actions=[
            LogInfo(msg="[Stage 5.5] Starting initial spin to help SLAM build map..."),
            start_initial_spin_node
        ]
    )
    ld.add_action(delayed_initial_spin)

    # ============================================================
    # 阶段 6: 等待 10 秒后启动可视化和通信 (所有核心功能就绪后)
    # ============================================================
    delayed_visualization_and_comm = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[Stage 6] Starting visualization and communication nodes..."),
            start_rviz_full,
            start_rviz_minimal,
            start_communication_node,
            start_tcp_client_node,
            start_explore_node
        ]
    )
    ld.add_action(delayed_visualization_and_comm)

    return ld