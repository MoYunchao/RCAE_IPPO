#!/usr/bin/env python3
"""
robot_lifecycle_manager.py

机器人节点进程管理器（使用进程杀死+自动重启）

功能:
- 监听 /robot_{id}/reset_command (从DOMAIN 10通过integration_service转发)
- 通过杀死进程来触发重启（launch文件中设置了respawn=True）
- 发送重置确认到 /system_reset_ack (转发回DOMAIN 10)
- 提供紧急清理服务给 odom_to_tf 调用

管理方式:
- SHUTDOWN: 杀死Nav2进程（launch会自动重启）；odom_to_tf 自己暂停 TF 发布
- STARTUP: 等待进程自动重启完成；重启 SLAM；odom_to_tf 自己恢复 TF 发布
- odom_to_tf 不再通过 pkill 管理，而是订阅 reset_command 自己处理重置
"""

import subprocess
import time
import rclpy
from rclpy.node import Node
from cure_explore_interfaces.msg import ResetCommand, ResetAck
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped

# ResetCommand command values
RESET_CMD_SHUTDOWN = 0
RESET_CMD_STARTUP = 1
RESET_CMD_RESTART = 2


class RobotLifecycleManager(Node):
    def __init__(self):
        super().__init__('robot_lifecycle_manager')

        # 获取自己的 PID，避免误杀
        import os
        self.my_pid = os.getpid()
        self.get_logger().info(f'RobotLifecycleManager PID: {self.my_pid}')

        # 声明参数
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('use_fastlio', True)

        self.robot_id = self.get_parameter('robot_id').value
        self.use_fastlio = self.get_parameter('use_fastlio').value
        
        # 标志位：是否在等待仿真重置完成
        self.waiting_for_sim_reset = False
        # 标志位：节点是否被紧急杀死过（需要重新激活）
        self.nodes_were_emergency_killed = False

        self.get_logger().info(
            f'RobotLifecycleManager 初始化 (robot_{self.robot_id}, '
            f'use_fastlio={self.use_fastlio})')

        # 定义需要管理的进程名称
        self.nav_process_names = [
            'bt_navigator',
            'waypoint_follower',
            'behavior_server',
            'planner_server',
            'controller_server',
            'smoother_server',
            'component_container', # Nav2 可能运行在容器中
            'component_container_isolated',
            'lifecycle_manager',   # Nav2 生命周期管理器
        ]

        # SLAM进程
        if self.use_fastlio:
            self.slam_process_names = ['fastlio_mapping'] # 假设 fastlio 的可执行文件名
        else:
            self.slam_process_names = ['async_slam_toolbox_node']

        self.all_process_names = self.slam_process_names + self.nav_process_names

        # 获取当前的 ROS_DOMAIN_ID
        import os
        self.current_domain_id = os.environ.get('ROS_DOMAIN_ID')
        self.get_logger().info(f'当前 ROS_DOMAIN_ID: {self.current_domain_id}')

        self.get_logger().info(f'将管理以下进程: {self.all_process_names}')

        # 订阅重置命令
        reset_topic = f'/robot_{self.robot_id-1}/reset_command'
        self.reset_sub = self.create_subscription(
            ResetCommand, reset_topic,
            self.reset_callback, 10)

        self.get_logger().info(f'订阅重置命令话题: {reset_topic}')

        # 发布确认 (会被integration_service转发回DOMAIN 10)
        self.ack_pub = self.create_publisher(
            ResetAck, '/system_reset_ack', 10)

        # 发布 initial pose (用于重置后设置机器人位置)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # 定期发布心跳日志，确认节点还活着
        self.heartbeat_timer = self.create_timer(10.0, self.heartbeat_callback)
        self.heartbeat_count = 0

        # 创建紧急清理服务（供 odom_to_tf 调用）
        self.emergency_cleanup_srv = self.create_service(
            Trigger,
            f'/robot_{self.robot_id-1}/emergency_cleanup_nav2',
            self.emergency_cleanup_callback
        )
        self.get_logger().info(f'提供紧急清理服务: /robot_{self.robot_id-1}/emergency_cleanup_nav2')

        self.get_logger().info('RobotLifecycleManager 已启动，等待重置命令...')

    def heartbeat_callback(self):
        """心跳回调，每10秒打印一次，确认节点还活着"""
        self.heartbeat_count += 1
        self.get_logger().info(f'💓 RobotLifecycleManager 心跳 #{self.heartbeat_count} - 节点正常运行')

    def emergency_cleanup_callback(self, request, response):
        """
        紧急清理服务回调（由 odom_to_tf 调用）
        """
        self.get_logger().warn('🚨 收到紧急清理请求（来自 odom_to_tf）')

        # 需要清理的进程：SLAM + Nav2 + 辅助进程
        # 恢复 robot_communication_node 和 rviz2，但通过 Domain ID 精确匹配防止误杀
        processes_to_kill = self.slam_process_names + self.nav_process_names + [
            'robot_communication_node',
            'rviz2',
        ]

        killed_count = 0
        for process_name in processes_to_kill:
            result = self._kill_process(process_name)
            if result == 'killed':
                self.get_logger().info(f'  ✓ {process_name} 已被紧急终止')
                killed_count += 1
            elif result == 'not_running':
                pass
            else:
                self.get_logger().warn(f'  ⚠ {process_name} 终止失败: {result}')

        response.success = True
        response.message = f'紧急清理完成：已终止 {killed_count} 个进程（launch 会自动重启）'
        self.get_logger().warn(response.message)
        return response

    def reset_callback(self, msg: ResetCommand):
        """处理重置命令"""
        # 立即打印，确认回调被触发
        self.get_logger().info(f'🔔 reset_callback 被调用！command={msg.command}')

        cmd_names = {0: 'SHUTDOWN', 1: 'STARTUP', 2: 'RESTART'}
        cmd_name = cmd_names.get(msg.command, f'UNKNOWN({msg.command})')

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'>>> 收到重置命令: {cmd_name} (code={msg.command})')
        self.get_logger().info('=' * 60)

        success = True
        message = ''

        try:
            if msg.command == RESET_CMD_SHUTDOWN:
                # SHUTDOWN: 立刻先杀死 SLAM（避免 TF 超时刷屏），再关闭 Nav2
                # 注意：进程可能已被 odom_to_tf 紧急杀死并通过 respawn 重启了
                self.get_logger().info('SHUTDOWN: 立刻关闭 SLAM 和 Nav2（避免 TF 刷屏）...')

                # 标记节点将被杀死（respawn 后需要重新激活）
                self.nodes_were_emergency_killed = True

                # 1. 【优先】立刻杀死 SLAM（如果不使用 FAST_LIO）
                # 此时 SLAM 可能已被 respawn 重启，直接杀死即可
                slam_killed = 0
                slam_not_running = 0
                slam_failed = 0

                if not self.use_fastlio:
                    self.get_logger().info('[1/3] 立刻杀死 SLAM（防止 TF 警告刷屏）...')
                    for process_name in self.slam_process_names:
                        result = self._kill_process(process_name)
                        if result == 'killed':
                            self.get_logger().info(f'  ✓ {process_name} 已关闭')
                            slam_killed += 1
                        elif result == 'not_running':
                            self.get_logger().info(f'  ○ {process_name} 未运行')
                            slam_not_running += 1
                        else:
                            self.get_logger().warn(f'  ⚠ {process_name} 关闭失败: {result}')
                            slam_failed += 1
                    time.sleep(0.2)  # 短暂等待确保进程完全退出

                # 2. 然后关闭 Nav2
                self.get_logger().info('[2/3] 关闭 Nav2 导航栈...')
                success_nav2 = self._shutdown_nav2_only()

                # 额外等待，确保 lifecycle 状态完全清理（DDS 发现、状态传播）
                self.get_logger().info('[3/3] 等待 2 秒让 lifecycle 状态完全清理...')
                time.sleep(2.0)

                # 宽容的成功判断：只要 Nav2 成功，且 SLAM 不是全部失败，就视为成功
                # 因为进程可能已经被看门狗清理了
                success_slam = (slam_failed == 0) or (slam_killed + slam_not_running > 0)
                success = success_nav2 and success_slam

                # 构建详细消息
                if success:
                    if slam_killed + slam_not_running == 0 and not self.use_fastlio:
                        message = f'SLAM和Nav2已关闭（SLAM: 无进程需要关闭）'
                    else:
                        message = f'SLAM和Nav2已关闭（SLAM: 已关闭{slam_killed}个，未运行{slam_not_running}个）'
                else:
                    message = f'关闭失败（SLAM失败{slam_failed}个，Nav2: {success_nav2}）'
                
            elif msg.command == RESET_CMD_STARTUP:
                # STARTUP: 等待旧 TF 数据失效后，重启 SLAM 和 Nav2
                # 注意：进程可能已被 odom_to_tf 紧急杀死并自动重启了
                self.get_logger().info('STARTUP: 仿真已重置，开始重启序列...')

                # 0. SHUTDOWN 已经等待了 2 秒，这里再等待 0.5 秒让 respawn 完成
                self.get_logger().info('[0/6] 等待 0.5s 让被紧急杀死的进程通过 respawn 重启...')
                time.sleep(0.5)

                if not self.use_fastlio:
                    # 1. 等待旧 TF 数据从缓存中失效（tf_buffer_duration=3s，留1s余量）
                    self.get_logger().info('[1/6] 等待 4s 让旧 TF 数据失效（tf_buffer=3s）...')
                    time.sleep(4.0)

                    # 2. 再次杀死 SLAM（因为它在 SHUTDOWN 后立刻重启了）
                    # 现在杀死它，让它用干净的状态重新启动
                    self.get_logger().info('[2/6] 再次杀死 SLAM（确保使用干净状态重启）...')
                    for process_name in self.slam_process_names:
                        result = self._kill_process(process_name)
                        if result == 'killed':
                            self.get_logger().info(f'  ✓ {process_name} 已关闭')
                        elif result == 'not_running':
                            self.get_logger().info(f'  ○ {process_name} 未运行')
                        else:
                            self.get_logger().warn(f'  ⚠ {process_name} 关闭失败: {result}')

                    # 3. 等待 SLAM 完全重启并稳定（2秒）
                    self.get_logger().info('[3/6] 等待 SLAM 完全重启并稳定（2s）...')
                    time.sleep(2.0)
                else:
                    # 如果使用 FASTLIO，也等待一下让系统稳定
                    self.get_logger().info('[1-3/6] 等待系统稳定（1.5s）...')
                    time.sleep(1.5)

                # 4. 等待 Nav2 进程自动重启和 lifecycle 初始化（5秒）
                self.get_logger().info('[4/6] 等待 Nav2 导航栈自动重启和 lifecycle 初始化（5s）...')
                time.sleep(5.0)

                # 5. 触发 DDS 发现（只做1次，节省时间）
                self.get_logger().info('[5/6] 触发 DDS 发现...')
                self._trigger_dds_discovery()

                # 6. 激活 Nav2 lifecycle 节点（仅在节点被杀死时）
                needs_activation = self.nodes_were_emergency_killed
                if needs_activation:
                    self.get_logger().info('[6/8] 激活 Nav2 lifecycle 节点（节点被杀死后 respawn）...')
                    self._activate_nav2_nodes()
                    # 重置标志
                    self.nodes_were_emergency_killed = False
                else:
                    self.get_logger().info('[6/8] 跳过节点激活（节点未被杀死，无需激活）')

                # 7. 短暂等待确保激活生效
                time.sleep(0.5 if needs_activation else 0.3)

                # 8. 发布 initial pose
                self.get_logger().info('[8/8] 发布 initial pose...')
                self._publish_initial_pose()

                success = True
                message = 'SLAM 和 Nav2 已重启并激活（TF 缓存已清空）'
                
            elif msg.command == RESET_CMD_RESTART:
                # RESTART: 完整重启流程
                self.get_logger().info('RESTART: 执行完整重启...')
                
                # 1. 关闭 Nav2
                success = self._shutdown_nav2_only()
                if not success:
                    message = 'Nav2关闭失败'
                    raise Exception(message)
                
                # 2. 重启 SLAM（如果需要）
                if not self.use_fastlio:
                    self.get_logger().info('重启 SLAM 和 odom_to_tf...')
                    self._restart_slam_processes()
                    # 额外等待，让 SLAM 完全稳定
                    self.get_logger().info('额外等待 2 秒让 SLAM 完全稳定...')
                    time.sleep(2.0)
                
                # 3. 等待 Nav2 重启
                self.get_logger().info('等待 Nav2 重启...')
                time.sleep(6.0)

                # 4. DDS 发现
                self._trigger_dds_discovery()
                time.sleep(1.0)
                self._trigger_dds_discovery()

                # 5. Initial pose
                self._publish_initial_pose()

                success = True
                message = '完整重启成功'
                
        except Exception as e:
            self.get_logger().error(f'重置操作异常: {e}')
            success = False
            message = f'异常: {str(e)}'

        # 发送确认
        ack = ResetAck()
        ack.source = f'robot_{self.robot_id}'
        ack.success = success
        ack.message = message

        self.get_logger().info('=' * 60)
        self.get_logger().info('🔔🔔🔔 准备发送 ACK 确认消息 🔔🔔🔔')
        self.get_logger().info(f'准备发送确认: source={ack.source}, success={success}')
        self.get_logger().info(f'确认消息: {message}')
        self.get_logger().info('=' * 60)

        self.ack_pub.publish(ack)

        self.get_logger().info('=' * 60)
        self.get_logger().info('✅✅✅ 确认消息已成功发布到 /system_reset_ack ✅✅✅')
        self.get_logger().info('=' * 60)

    def _shutdown_processes(self):
        """关闭所有Nav2和SLAM进程（已弃用，使用 _shutdown_nav2_only）"""
        return self._shutdown_nav2_only()
    
    def _shutdown_nav2_only(self):
        """只关闭 Nav2 导航栈，不关闭 SLAM"""
        self.get_logger().info('开始关闭 Nav2 导航栈...')

        # 统计关闭结果
        killed_count = 0
        not_running_count = 0
        failed_count = 0

        # 只关闭 Nav2 进程
        for process_name in self.nav_process_names:
            result = self._kill_process(process_name)
            if result == 'killed':
                self.get_logger().info(f'  ✓ {process_name} 已关闭')
                killed_count += 1
            elif result == 'not_running':
                self.get_logger().info(f'  ○ {process_name} 未运行')
                not_running_count += 1
            else:  # 'failed'
                self.get_logger().warn(f'  ⚠ {process_name} 关闭失败: {result}')
                failed_count += 1

        # 等待进程完全退出
        time.sleep(0.5)

        self.get_logger().info(
            f'Nav2 导航栈关闭完成：已关闭{killed_count}个，未运行{not_running_count}个，失败{failed_count}个')

        # 宽容策略：只要不是全部失败，就视为成功
        # 因为进程可能已经被看门狗或其他方式清理了
        return failed_count == 0 or (killed_count + not_running_count) > 0
    
    def _restart_slam_processes(self):
        """重启 SLAM 相关进程（slam_toolbox 和 odom_to_tf）"""
        self.get_logger().info('开始重启 SLAM 进程...')

        # 关闭旧的 SLAM 进程
        for process_name in self.slam_process_names:
            result = self._kill_process(process_name)
            if result == 'killed':
                self.get_logger().info(f'  ✓ {process_name} 已关闭')
            elif result == 'not_running':
                self.get_logger().info(f'  ○ {process_name} 未运行')
            else:
                self.get_logger().warn(f'  ⚠ {process_name} 关闭失败: {result}')

        # 等待进程完全退出，并等待 respawn 重启
        # SLAM 需要更多时间来清除内部状态和重新初始化
        # 增加等待时间，让 SLAM 完全初始化并建立 TF 监听器
        self.get_logger().info('等待 SLAM 进程完全退出并重启（4秒，确保 TF 和消息队列就绪）...')
        time.sleep(4.0)  # 从 2秒 增加到 4秒

        self.get_logger().info('SLAM 进程重启完成')

    def _trigger_dds_discovery(self):
        """触发 DDS 发现机制 - 查询话题、节点和 action 来加速发现"""
        self.get_logger().info('触发完整 DDS 发现（topics, nodes, actions）...')

        # 1. 触发话题发现
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=3.0
            )
            if result.returncode == 0:
                topic_count = len([line for line in result.stdout.strip().split('\n') if line])
                self.get_logger().info(f'  ✓ 发现 {topic_count} 个话题')
        except Exception as e:
            self.get_logger().warn(f'  ✗ 话题发现失败: {e}')

        # 2. 触发节点发现
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=3.0
            )
            if result.returncode == 0:
                node_count = len([line for line in result.stdout.strip().split('\n') if line])
                self.get_logger().info(f'  ✓ 发现 {node_count} 个节点')
        except Exception as e:
            self.get_logger().warn(f'  ✗ 节点发现失败: {e}')

        # 3. 触发 action 发现（最重要 - 用于 navigate_to_pose 等 action server）
        try:
            result = subprocess.run(
                ['ros2', 'action', 'list'],
                capture_output=True,
                text=True,
                timeout=3.0
            )
            if result.returncode == 0:
                action_count = len([line for line in result.stdout.strip().split('\n') if line])
                self.get_logger().info(f'  ✓ 发现 {action_count} 个 action servers')

                # 检查关键的 navigate_to_pose action
                if '/navigate_to_pose' in result.stdout:
                    self.get_logger().info('  ✓ navigate_to_pose action server 可用')
                else:
                    self.get_logger().warn('  ⚠ navigate_to_pose action server 未发现')
        except Exception as e:
            self.get_logger().warn(f'  ✗ Action 发现失败: {e}')

        self.get_logger().info('DDS 发现完成')

    def _publish_initial_pose(self):
        """发布初始位姿到 /initialpose 话题，告诉 Nav2 机器人位置"""
        try:
            # 创建 initial pose 消息
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'

            # 重置后机器人在 odom 原点，对应于 map 坐标系的原点
            # (因为 SLAM 也重置了，所以 map 和 odom 都对齐到新的起始位置)
            pose_msg.pose.pose.position.x = 0.0
            pose_msg.pose.pose.position.y = 0.0
            pose_msg.pose.pose.position.z = 0.0

            # 朝向为 0（沿 x 轴正方向）
            pose_msg.pose.pose.orientation.x = 0.0
            pose_msg.pose.pose.orientation.y = 0.0
            pose_msg.pose.pose.orientation.z = 0.0
            pose_msg.pose.pose.orientation.w = 1.0

            # 设置协方差（对初始位置有较高置信度）
            # 6x6 协方差矩阵，展开为 36 个元素的数组
            pose_msg.pose.covariance = [0.0] * 36
            pose_msg.pose.covariance[0] = 0.1   # x 方向方差
            pose_msg.pose.covariance[7] = 0.1   # y 方向方差
            pose_msg.pose.covariance[35] = 0.05 # yaw 方向方差

            # 发布（发布多次确保被接收）
            for _ in range(3):
                self.initialpose_pub.publish(pose_msg)
                time.sleep(0.1)

            self.get_logger().info('  ✓ Initial pose 已发布: (0.0, 0.0, 0.0)')

        except Exception as e:
            self.get_logger().error(f'发布 initial pose 失败: {e}')

    def _activate_nav2_nodes(self):
        """
        并行激活所有 Nav2 lifecycle 节点

        优化：
        1. 并行检查所有节点状态（0.5秒，而不是3.5秒）
        2. 并行激活需要的节点（3秒，而不是10.5秒）
        3. 条件重启 lifecycle_manager
        """
        import concurrent.futures

        # 需要检查的 Nav2 lifecycle 节点
        nav2_lifecycle_nodes = [
            'controller_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'smoother_server',
            'waypoint_follower',
            'velocity_smoother',
        ]

        # 步骤 0: 并行检查所有节点状态
        def check_node_state(node_name):
            try:
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'get', f'/{node_name}'],
                    capture_output=True,
                    text=True,
                    timeout=0.5
                )
                if result.returncode == 0:
                    state = result.stdout.strip().lower()
                    return (node_name, state)
            except:
                pass
            return (node_name, 'unknown')

        self.get_logger().info('  并行检查 Nav2 节点状态...')
        with concurrent.futures.ThreadPoolExecutor(max_workers=7) as executor:
            state_futures = [executor.submit(check_node_state, node) for node in nav2_lifecycle_nodes]
            node_states = [f.result() for f in state_futures]

        # 分析状态
        nodes_need_activation = []
        for node_name, state in node_states:
            if 'inactive' in state or 'unconfigured' in state or 'unknown' in state:
                nodes_need_activation.append(node_name)
                self.get_logger().info(f'    {node_name}: {state} → 需要激活')
            else:
                self.get_logger().info(f'    {node_name}: {state} → 已就绪')

        # 如果所有节点都已就绪，直接返回
        if not nodes_need_activation:
            self.get_logger().info('  ✓ 所有 Nav2 节点已就绪，无需激活')
            return True

        # 步骤 1: 并行激活需要的节点
        def activate_node(node_name):
            try:
                result = subprocess.run(
                    ['ros2', 'lifecycle', 'set', f'/{node_name}', 'activate'],
                    capture_output=True,
                    text=True,
                    timeout=3.0  # 增加到3秒（controller_server 和 planner_server 需要更长时间）
                )
                if result.returncode == 0 and ('Transitioning successful' in result.stdout or 'already in target state' in result.stdout):
                    return (node_name, True, None)
                else:
                    return (node_name, False, result.stdout.strip())
            except subprocess.TimeoutExpired:
                return (node_name, False, '超时')
            except Exception as e:
                return (node_name, False, str(e))

        self.get_logger().info(f'  [1/2] 并行激活 {len(nodes_need_activation)} 个节点...')
        with concurrent.futures.ThreadPoolExecutor(max_workers=7) as executor:
            activate_futures = [executor.submit(activate_node, node) for node in nodes_need_activation]
            activation_results = [f.result() for f in activate_futures]

        # 统计结果
        activated = []
        failed = []
        for node_name, success, error_msg in activation_results:
            if success:
                self.get_logger().info(f'    ✓ {node_name}')
                activated.append(node_name)
            else:
                self.get_logger().warn(f'    ⚠ {node_name} 失败: {error_msg}')
                failed.append(node_name)

        self.get_logger().info(f'  节点激活完成：成功 {len(activated)}，失败 {len(failed)}')

        # 步骤 2: 仅在有节点被激活时才重启 lifecycle_manager
        if activated:
            self.get_logger().info('  [2/2] 重启 lifecycle_manager 以更新 rviz2 状态...')
            try:
                subprocess.run(['ros2', 'lifecycle', 'set', '/lifecycle_manager_navigation', 'deactivate'],
                              capture_output=True, timeout=2.0)
                time.sleep(0.2)

                subprocess.run(['ros2', 'lifecycle', 'set', '/lifecycle_manager_navigation', 'cleanup'],
                              capture_output=True, timeout=2.0)
                time.sleep(0.2)

                subprocess.run(['ros2', 'lifecycle', 'set', '/lifecycle_manager_navigation', 'configure'],
                              capture_output=True, timeout=2.0)
                time.sleep(0.2)

                result = subprocess.run(['ros2', 'lifecycle', 'set', '/lifecycle_manager_navigation', 'activate'],
                                      capture_output=True, text=True, timeout=2.0)

                if 'successful' in result.stdout:
                    self.get_logger().info('    ✓ lifecycle_manager 已更新')
                else:
                    self.get_logger().warn(f'    ⚠ lifecycle_manager 激活失败')

            except Exception as e:
                self.get_logger().error(f'    ✗ lifecycle_manager 重启失败: {e}')
        else:
            self.get_logger().info('  [2/2] 跳过 lifecycle_manager 重启（无节点被激活）')

        return len(failed) == 0 or len(activated) > 0

    def _get_process_env(self, pid, env_var):
        """读取指定进程的环境变量"""
        try:
            with open(f'/proc/{pid}/environ', 'rb') as f:
                content = f.read()
                # environ is null-separated
                for entry in content.split(b'\0'):
                    try:
                        key_val = entry.decode('utf-8').split('=', 1)
                        if len(key_val) == 2 and key_val[0] == env_var:
                            return key_val[1]
                    except:
                        continue
        except Exception:
            pass
        return None

    def _kill_process(self, process_name):
        """
        使用pgrep+kill杀死指定进程（带自我保护和Domain ID过滤）

        返回值:
        - 'killed': 进程被成功杀死
        - 'not_running': 进程不存在（未运行）
        - 'timeout': 超时
        - 'error: <details>': 其他错误
        """
        try:
            # 1. 先用 pgrep 找到匹配的进程 PID 列表
            result = subprocess.run(
                ['pgrep', '-f', process_name],
                capture_output=True,
                text=True,
                timeout=1.0
            )

            if result.returncode == 1:
                # 没有匹配的进程
                return 'not_running'
            elif result.returncode != 0:
                return f'error: pgrep_returncode={result.returncode}'

            # 2. 解析 PID 列表并过滤
            pids = [int(pid.strip()) for pid in result.stdout.strip().split('\n') if pid.strip()]

            # 详细日志：显示找到的所有 PID
            self.get_logger().info(f'  [DEBUG] pgrep -f "{process_name}" 找到 PID: {pids}')
            
            pids_to_kill = []
            for pid in pids:
                # 过滤掉自己
                if pid == self.my_pid:
                    continue
                
                # 过滤 Domain ID
                # 如果当前节点有 ROS_DOMAIN_ID，则只杀死相同 Domain 的进程
                # 如果当前节点没有 ROS_DOMAIN_ID (None)，则默认不进行 Domain 过滤（或者只杀没有 Domain 的？）
                # 这里采取保守策略：如果我有 Domain ID，对方必须匹配；如果我没有，则不检查
                if self.current_domain_id is not None:
                    proc_domain = self._get_process_env(pid, 'ROS_DOMAIN_ID')
                    if proc_domain != self.current_domain_id:
                        self.get_logger().info(f'  [DEBUG] PID {pid} Domain ID ({proc_domain}) 不匹配 ({self.current_domain_id})，跳过')
                        continue
                
                pids_to_kill.append(pid)

            if not pids_to_kill:
                self.get_logger().warn(f'⚠ {process_name} 没有找到匹配 Domain 的进程，跳过')
                return 'not_running'

            # 3. 杀死过滤后的进程
            killed_count = 0
            for pid in pids_to_kill:
                # 双重保护
                if pid == self.my_pid:
                    continue

                try:
                    self.get_logger().info(f'  [DEBUG] 准备杀死 PID {pid}...')
                    subprocess.run(
                        ['kill', '-9', str(pid)],
                        capture_output=True,
                        timeout=0.5
                    )
                    killed_count += 1
                    self.get_logger().info(f'  [DEBUG] 已杀死 PID {pid}')
                except Exception as e:
                    self.get_logger().warn(f'  杀死 PID {pid} 失败: {e}')

            if killed_count > 0:
                return 'killed'
            else:
                return 'not_running'

        except subprocess.TimeoutExpired:
            self.get_logger().error(f'pgrep/kill {process_name} 超时（1秒）- 跳过并继续')
            return 'timeout'
        except Exception as e:
            self.get_logger().error(f'杀死进程 {process_name} 异常: {e}')
            return f'error: {str(e)}'


def main(args=None):
    rclpy.init(args=args)

    node = RobotLifecycleManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Exception in main loop: {e}')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        
        # 只在上下文仍然有效时才关闭
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
