#!/usr/bin/env python3
"""
test_recovery_manager.py

简单测试 recovery_manager 的功能
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time

class RecoveryTester(Node):
    def __init__(self):
        super().__init__('recovery_tester')
        
        # 订阅恢复状态
        self.status_sub = self.create_subscription(
            String,
            '/robot_1/recovery_status',
            self.status_callback,
            10
        )
        
        # 订阅恢复目标
        self.recovery_goal_sub = self.create_subscription(
            PoseStamped,
            '/robot_1/recovery_goal',
            self.recovery_goal_callback,
            10
        )
        
        self.current_status = None
        self.recovery_goal_count = 0
        
        self.get_logger().info('🧪 Recovery Manager Tester Started')
        self.get_logger().info('Listening to /robot_1/recovery_status and /robot_1/recovery_goal')

    def status_callback(self, msg: String):
        if msg.data != self.current_status:
            self.current_status = msg.data
            self.get_logger().info(f'📊 Status changed to: {msg.data}')

    def recovery_goal_callback(self, msg: PoseStamped):
        self.recovery_goal_count += 1
        pos = msg.pose.position
        self.get_logger().info(
            f'🎯 Recovery goal #{self.recovery_goal_count} received: '
            f'({pos.x:.2f}, {pos.y:.2f}) in frame {msg.header.frame_id}'
        )

def main(args=None):
    rclpy.init(args=args)
    
    tester = RecoveryTester()
    
    print('\n' + '='*60)
    print('Recovery Manager Test Instructions:')
    print('='*60)
    print('1. Make sure your robot simulation is running')
    print('2. Launch the cure_explore system with recovery_manager')
    print('3. Manually block the robot\'s path to trigger recovery')
    print('4. Watch for status changes and recovery goals')
    print('5. Press Ctrl+C to exit')
    print('='*60 + '\n')
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print('\n\n' + '='*60)
        print('Test Summary:')
        print('='*60)
        print(f'Total recovery goals received: {tester.recovery_goal_count}')
        print(f'Final status: {tester.current_status}')
        print('='*60)
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
