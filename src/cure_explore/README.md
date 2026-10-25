# Multi-Robot Exploration Package

基于 CURE (Cooperative Unified Robotic Exploration) 算法的多机器人协同探索 ROS 2 包。

## 功能特性

- ✅ 多机器人地图融合
- ✅ 前沿边界检测 (Frontier Detection)
- ✅ Voronoi 区域划分
- ✅ 目标点分配 (Target Assignment)
- ✅ 路径可视化
- ✅ 机器人位置发布
- ✅ 目标点中继

## 包结构

```
cure_explore/
├── config/
│   └── robot_offsets.yaml          # 机器人初始偏移配置
├── launch/
│   └── cure_explore.launch.py      # 主启动文件
├── cure_explore/
│   ├── __init__.py                 # 包初始化
│   ├── frontier_detector.py        # 前沿边界检测节点
│   ├── goal_relay.py               # 目标点中继节点
│   ├── map_merger.py               # 地图融合节点
│   ├── map_tf_broadcaster.py       # 地图TF广播节点
│   ├── path_visualizer.py          # 路径可视化节点
│   ├── robot_position_publisher.py # 机器人位置发布节点
│   ├── target_assigner.py          # 目标分配节点
│   └── voronoi_centroid.py         # Voronoi质心计算节点
├── rviz/
│   └── cure_display.rviz           # RViz 配置
├── resource/
│   └── cure_explore        # 包标记文件
├── package.xml                     # 包元数据
├── setup.py                        # Python 包设置
├── setup.cfg                       # 安装配置
└── README.md                       # 本文件
```

## 依赖项

### ROS 2 依赖
- `rclpy`
- `std_msgs`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `visualization_msgs`
- `nav2_msgs`
- `tf2_ros`
- `tf2_geometry_msgs`

### Python 依赖
- `numpy`
- `opencv-python` (python3-opencv)
- `scipy` (python3-scipy)
- `pyyaml` (python3-yaml)

## 编译

```bash
cd ~/SenseBeetle
colcon build --packages-select cure_explore --symlink-install
source install/setup.bash
```

## 使用方法

### 启动探索系统

```bash
# 启动完整的多机器人探索系统
ros2 launch cure_explore cure_explore.launch.py
```

### 单独运行节点

```bash
# 地图融合节点
ros2 run cure_explore map_merger

# 前沿边界检测节点
ros2 run cure_explore frontier_detector

# 目标分配节点
ros2 run cure_explore target_assigner

# 其他节点...
```

## 节点说明

### 1. map_merger
- **订阅**: `/robot_*/map` (各机器人的局部地图)
- **发布**: `/global_map` (融合后的全局地图)
- **功能**: 实时融合所有机器人的地图

### 2. frontier_detector
- **订阅**: `/global_map`
- **发布**: `/frontiers`, `/frontier_clusters_vis`
- **功能**: 检测已知与未知区域的边界

### 3. voronoi_centroid
- **订阅**: `/global_map`, `/frontiers`
- **发布**: `/centroids_viz`
- **功能**: 基于Voronoi图计算前沿质心

### 4. target_assigner
- **订阅**: `/global_map`, `/centroids_viz`, `/robot_*/pose`
- **发布**: `/robot_*/waypoint`
- **功能**: 为每个机器人分配探索目标

### 5. robot_position_publisher
- **订阅**: `/robot_*/odom` 或 `/robot_*/map`
- **发布**: `/robot_*/pose`
- **功能**: 统一机器人位置发布格式

### 6. goal_relay
- **订阅**: `/robot_*/waypoint`
- **发布**: `/robot_*/goal_pose`
- **功能**: 中继目标点给导航系统

### 7. path_visualizer
- **订阅**: `/robot_*/plan`
- **发布**: `/path_viz`
- **功能**: 可视化所有机器人的规划路径

### 8. map_tf_broadcaster
- **功能**: 广播地图坐标系TF变换

## 配置参数

编辑 `config/robot_offsets.yaml` 设置机器人初始位置：

```yaml
robot_offsets:
  robot_1:
    x: -0.5
    y: 0.0
    z: 0.0
  robot_2:
    x: 0.5
    y: 0.0
    z: 0.0
```

## RViz 可视化

使用提供的 RViz 配置文件：

```bash
rviz2 -d install/cure_explore/share/cure_explore/rviz/cure_display.rviz
```

## 许可证

Apache-2.0

## 维护者

Sunny <sunny@example.com>
