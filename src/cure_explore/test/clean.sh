# 1. 杀掉核心管理节点
pkill -f cure_lifecycle
pkill -f system_reset

# 2. 杀掉探索和地图相关节点
pkill -f exploration_monitor
pkill -f frontier
pkill -f map_merger

# 3. 杀掉语义和 TF 相关节点
pkill -f semantic
pkill -f dynamic_tf

# 4. 杀掉机器人相关的具体节点
pkill -f goal_transformer
pkill -f robot_position
pkill -f safety_shield
pkill -f pose_config_server