#!/usr/bin/env python3
"""
测试 frontier_cost_service 是否正常工作的简单示例

注意：自动设置 ROS_DOMAIN_ID=10
订阅 /frontiers 话题，使用真实的前沿点测试服务
"""

import os
# 在导入 rclpy 之前设置 domain ID
os.environ['ROS_DOMAIN_ID'] = '10'

import rclpy
from rclpy.node import Node
from cure_explore_interfaces.srv import ComputeFrontierCosts
from cure_explore_interfaces.msg import FrontierArray
from geometry_msgs.msg import Point

class FrontierCostTester(Node):
    """测试前沿点代价计算服务"""
    
    def __init__(self):
        super().__init__('frontier_cost_tester')
        
        # 前沿点数据
        self.frontiers = None
        self.frontier_received = False
        
        # 创建服务客户端
        self.client = self.create_client(ComputeFrontierCosts, '/compute_frontier_costs')
        
        # 订阅前沿点话题
        self.frontier_sub = self.create_subscription(
            FrontierArray,
            '/frontiers',
            self.frontier_callback,
            10
        )
        
        self.get_logger().info('等待前沿点数据和服务...')
    
    def frontier_callback(self, msg):
        """接收前沿点数据"""
        if not self.frontier_received and len(msg.clusters) > 0:
            self.frontiers = msg.clusters
            self.frontier_received = True
            self.get_logger().info(f'✓ 收到 {len(self.frontiers)} 个前沿点簇')

def test_service():
    """测试服务能否正常调用"""
    rclpy.init()
    tester = FrontierCostTester()

    # 等待服务可用
    print("等待服务 /compute_frontier_costs...")
    if not tester.client.wait_for_service(timeout_sec=5.0):
        print("❌ 服务超时，请确保 frontier_cost_service 正在运行")
        tester.destroy_node()
        rclpy.shutdown()
        return False

    print("✓ 服务可用")

    # 等待接收前沿点数据
    print("等待前沿点数据 /frontiers...")
    timeout = 10.0
    start_time = tester.get_clock().now()
    
    while not tester.frontier_received:
        rclpy.spin_once(tester, timeout_sec=0.1)
        elapsed = (tester.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > timeout:
            print("❌ 超时未收到前沿点数据，请确保 frontier_detector 正在运行")
            tester.destroy_node()
            rclpy.shutdown()
            return False
    
    # 使用接收到的前沿点创建服务请求
    request = ComputeFrontierCosts.Request()
    # 只取前3个前沿点进行测试
    request.frontiers = tester.frontiers

    print(f"\n发送请求：计算 {len(request.frontiers)} 个前沿点到所有机器人的距离...")

    # 调用服务并计时
    import time
    start_time = time.time()
    future = tester.client.call_async(request)
    rclpy.spin_until_future_complete(tester, future, timeout_sec=10.0)
    end_time = time.time()
    elapsed_time = end_time - start_time

    if future.result() is not None:
        response = future.result()

        if response.success:
            print(f"\n✓ {response.message}")
            print(f"⏱️  服务调用耗时: {elapsed_time:.3f} 秒\n")

            # 显示结果 - 一次性得到所有数据
            print("=" * 60)
            print("前沿点到所有机器人的距离矩阵：")
            print("=" * 60)

            for fc in response.frontier_costs:
                print(f"\n📍 前沿点 {fc.frontier_id} @ ({fc.centroid.x:.1f}, {fc.centroid.y:.1f})")
                print(f"   到各机器人的距离：")

                for cost in fc.costs:
                    if cost.astar_distance >= 0:
                        print(f"   🤖 机器人 {cost.robot_id}: "
                              f"欧式={cost.euclidean_distance:.2f}m, "
                              f"A*={cost.astar_distance:.2f}m, "
                              f"代价={cost.astar_cost:.1f}")
                    else:
                        print(f"   🤖 机器人 {cost.robot_id}: "
                              f"欧式={cost.euclidean_distance:.2f}m, "
                              f"A*=不可达")

            print("\n" + "=" * 60)
            print("✓ 一次调用成功获取了所有前沿点到所有机器人的距离！")
            print("=" * 60)

            tester.destroy_node()
            rclpy.shutdown()
            return True
        else:
            print(f"❌ 服务调用失败: {response.message}")
    else:
        print("❌ 服务调用超时")

    tester.destroy_node()
    rclpy.shutdown()
    return False

if __name__ == '__main__':
    print("\n" + "=" * 60)
    print("测试 frontier_cost_service 服务")
    print("=" * 60)
    print(f"\n✓ ROS_DOMAIN_ID 已设置为: {os.environ.get('ROS_DOMAIN_ID', 'default')}")
    print("\n说明：")
    print("- 订阅 /frontiers 话题获取真实前沿点数据")
    print("- 一次服务调用可以获取所有前沿点到所有机器人的距离")
    print("- 不需要每个机器人单独调用")
    print("- 返回完整的距离矩阵\n")

    success = test_service()

    if success:
        print("\n✓ 测试通过！服务工作正常。\n")
    else:
        print("\n❌ 测试失败！请检查：")
        print("  1. frontier_cost_service 是否正在运行")
        print("  2. frontier_detector 是否正在运行（提供前沿点）")
        print("  3. map_merger 是否正在运行（提供地图）")
        print("  4. robot_position_publisher 是否正在运行（提供机器人位置）\n")
