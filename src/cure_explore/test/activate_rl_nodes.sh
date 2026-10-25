#!/bin/bash
# Activate LifecycleNodes for rl_training.launch.py

echo "Activating pose_config_server..."
ros2 lifecycle set /pose_config_server configure
ros2 lifecycle set /pose_config_server activate

echo "Activating dynamic_tf_broadcaster..."
ros2 lifecycle set /dynamic_tf_broadcaster configure
ros2 lifecycle set /dynamic_tf_broadcaster activate

echo "Activating system_reset_server..."
ros2 lifecycle set /system_reset_server configure
ros2 lifecycle set /system_reset_server activate

echo "All nodes activated!"
