#!/bin/bash
# test_system_reset.sh - 测试系统重置服务

export ROS_DOMAIN_ID=10

echo "=========================================="
echo "测试系统重置服务"
echo "=========================================="
echo ""

# echo "1️⃣ 测试基本重置 (不更新位置，不重置仿真)"
# echo "------------------------------------------"
# ros2 service call /system_reset cure_explore_interfaces/srv/SystemReset \
#     "{reset_simulation: false, new_poses: []}"

# echo ""
# echo ""
# echo "2️⃣ 测试带位置更新的重置"
# echo "------------------------------------------"
# ros2 service call /system_reset cure_explore_interfaces/srv/SystemReset \
#     "{reset_simulation: false, new_poses: [{robot_id: 'robot_1', x: 0.0, y: 0.0, yaw: 0.0}, {robot_id: 'robot_2', x: -0.5, y: 0.0, yaw: 0.0}]}"

echo ""
echo ""
echo "3️⃣ 测试带仿真重置的重置"
echo "------------------------------------------"
ros2 service call /system_reset cure_explore_interfaces/srv/SystemReset \
    "{reset_simulation: true, new_poses: [{robot_id: 'robot_1', x: 10.0, y: 0.0, yaw: 0.0}, {robot_id: 'robot_2', x: 0.0, y: 10.0, yaw: 0.0}]}"
sleep 5

echo ""
echo ""
echo "3️⃣ 测试带仿真重置的重置"
echo "------------------------------------------"
ros2 service call /system_reset cure_explore_interfaces/srv/SystemReset \
    "{reset_simulation: true, new_poses: [{robot_id: 'robot_1', x: 0.0, y: 5.0, yaw: 0.0}, {robot_id: 'robot_2', x: 5.0, y: 0.0, yaw: 0.0}]}"

echo ""ros2 service call /system_reset cure_explore_interfaces/srv/SystemReset \
    "{reset_simulation: true, new_poses: [{robot_id: 'robot_1', x: 10.0, y: 0.0, yaw: 0.0}, {robot_id: 'robot_2', x: 0.0, y: 10.0, yaw: 0.0}, {robot_id: 'robot_3', x: 0.0, y: 0.0, yaw: 0.0}]}"
echo "=========================================="
echo "测试完成"
echo "=========================================="
