# RCAE_IPPO 推理仓库

IPPO 单次推理入口为 `start_all_ippo.sh`，八次批量实验入口为 `run_ippo_8runs.py`。
模型已随仓库提供，拉取、编译、运行和结果目录见 [IPPO_RUN.md](IPPO_RUN.md)。

以下保留原工作空间说明，其他历史脚本不代表本仓库的 IPPO 主入口。

---

# SenseSimulation_fix

基于 **ROS 2 Humble + Gazebo Classic** 的三机器人集中式协同探索仿真工程。该仓库面向室内未知环境探索场景，集成了：

- 三机器人仿真与统一启动
- 每台机器人独立建图、定位与导航
- 全局地图融合
- 语义地图生成
- 前沿提取与聚类
- 模糊决策评分
- 多机器人目标分配
- 达到覆盖率阈值后的自动停止与指标落盘

当前仓库的主启动入口是根目录下的 **`start_all.sh`**。如果你平时说的 `start_all.bash` 指的就是这个统一启动流程，可以直接使用本文档替换原 README。

---

## 1. 项目概述

该工程采用 **集中式多机器人探索架构**。

系统运行时：

1. Gazebo 中生成 3 台机器人；
2. 每台机器人独立运行 SLAM、局部导航和任务执行；
3. 服务端融合多机器人局部地图，生成 `/global_map`；
4. 基于全局地图和语义地图提取前沿，并计算每个前沿对不同机器人的距离；
5. 各机器人分别对前沿做模糊评分；
6. 中央目标分配节点为每台机器人选择互不冲突的目标点；
7. 导航节点执行任务，直到探索覆盖率达到设定阈值；
8. 系统自动发布停止信号，并将指标写入 CSV。

默认流程中启用的是 **Oracle 语义地图节点**，即直接从 `model_semantic.world` 中读取语义区域并发布语义值/风险图；仓库中虽然保留了 CLIP 相关代码，但 `start_all.sh` 对应的完整启动链路默认并不是 CLIP 在线语义识别流程。

---

## 2. 主要功能

### 2.1 三机器人仿真启动

由 `simulation_bringup` 统一启动 Gazebo、机器人模型、初始位姿发布、SLAM、探索、模糊决策、目标分配和 RViz。

默认世界文件：

- `src/simulation_bringup/world/model/model_semantic.world`

默认 RViz 配置：

- `src/simulation_bringup/config/fuzzy_decision.rviz`

### 2.2 单机器人局部建图与导航

每台机器人默认包含以下能力：

- `slam_toolbox` 异步建图
- `pointcloud_to_laserscan` 点云转激光
- `odom_to_tf.py` 里程计到 TF 的转换
- `map_postprocessor` 对局部地图做后处理
- Nav2 导航栈
- `minimal_task_navigator` 执行分配到的任务点

从参数文件和 launch 配置来看，当前局部导航使用的是 **Nav2 MPPI 控制器配置**。

### 2.3 全局地图融合

`cure_explore_cpp/map_merger_node` 订阅各机器人局部地图，生成全局地图：

- 输出话题：`/global_map`

同时系统通过 `dynamic_tf_broadcaster` 维护 `global_map -> robot_i/map` 的动态 TF。

### 2.4 语义地图生成

完整启动流程默认启动：

- `clip_semantic/oracle_semantic_mapper`

它会读取世界文件中的语义标记区域，发布：

- `/semantic_grid_float`
- `/semantic_risk_grid_float`

这意味着当前主流程下，语义信息是由仿真真值生成的，而不是依赖在线视觉识别。

### 2.5 前沿提取与可视化

`rgbd_to_map/frontier_detector` 基于 `/global_map` 进行前沿检测与 DBSCAN 聚类，并结合语义值、语义风险和障碍密度计算前沿属性。

主要输出：

- `/frontiers_info`：前沿扩展信息
- `/frontiers_markers`：RViz 可视化 Marker

### 2.6 模糊决策评分

每个机器人运行一个 `fuzzy_decision_node`，根据前沿的以下信息综合评分：

- 距离
- 语义得分
- 探索增益
- 风险水平

输出话题：

- `/robot_i/frontier_priority_fuzzy`

### 2.7 多机器人目标分配

`goal_select/multi_robot_goal_selector_node` 接收 3 台机器人各自的前沿评分结果，按分数排序并进行去冲突分配，最终向每台机器人发布任务点：

- `/robot_0/task_point`
- `/robot_1/task_point`
- `/robot_2/task_point`

### 2.8 覆盖率监督、自动停止与指标记录

`simulation_bringup/exploration_supervisor.py` 会持续统计 ROI 区域内的已知栅格占比。

默认行为：

- 覆盖率目标：`0.95`
- 达标保持时间：`3.0 s`
- 最短运行时间：`20.0 s`
- 达标后发布：`/exploration_stop`
- 同时将指标写入：`data/metrics/energy_mission_metrics.csv`

CSV 中记录的核心数据包括：

- 运行时间
- 覆盖率
- ROI 已知栅格数
- 每台机器人累计路径长度
- 总路径长度
- 停止原因

---

## 3. 仓库结构

```text
SenseSimulation_fix/
├── start_all.sh                  # 一键启动完整系统
├── toBuild.sh                    # 清理并重新编译
├── simulation_env.sh             # 历史单独启动脚本之一
├── robot_0_slam.sh               # 历史单独启动脚本之一
├── robot_1_slam.sh
├── robot_2_slam.sh
├── map_merge.sh
├── robot_0_fuzzy.sh
├── robot_1_fuzzy.sh
├── robot_2_fuzzy.sh
├── data/                         # 结果输出目录
├── logs/                         # 运行日志目录
└── src/
    ├── simulation_bringup/       # 仿真、总 launch、RViz、世界文件、监督脚本
    ├── robot_navigation/         # 单机器人导航相关配置与节点
    ├── multi_robot_navigation/   # 任务点导航执行
    ├── goal_select/              # 多机器人目标分配
    ├── fuzzy_decision/           # 模糊评分
    ├── rgbd_to_map/              # 前沿检测及部分地图处理
    ├── cure_explore/             # 动态 TF 广播等 Python 节点
    ├── cure_explore_cpp/         # C++ 地图融合
    ├── clip_semantic/            # Oracle/CLIP 语义相关节点
    ├── map_postprocessor/        # 局部地图后处理
    └── ...
```

---

## 4. 运行环境

建议环境：

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- colcon

代码中明确依赖或直接使用到的典型组件包括：

- `slam_toolbox`
- `nav2_*`
- `gazebo_ros`
- `pointcloud_to_laserscan`
- `tf2_ros`
- `xacro`
- OpenCV / PCL 相关依赖

如果是全新环境，推荐先在工作空间根目录执行 ROS 依赖安装：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

---

## 5. 编译

在仓库根目录执行：

```bash
bash toBuild.sh
```

该脚本当前内容等价于：

```bash
rm -rf build log install
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

编译完成后记得加载环境：

```bash
source install/setup.bash
```

---

## 6. 启动方式

### 6.1 推荐方式：统一启动

在仓库根目录执行：

```bash
bash start_all.sh
```

`start_all.sh` 当前会执行：

```bash
source install/setup.bash
ros2 launch simulation_bringup full_exploration_system.launch.py
```

### 6.2 统一启动流程的内部时序

`full_exploration_system.launch.py` 中的启动顺序大致如下：

1. 立即启动仿真环境；
2. 约 6 秒后同时启动 3 台机器人的 SLAM / Nav2 相关链路；
3. 约 16 秒后启动多机器人探索公共部分：
   - 动态 TF
   - 地图融合
   - Oracle 语义地图
   - 前沿检测
   - 覆盖率监督
4. 约 20 秒后启动 3 台机器人的模糊决策与任务导航；
5. 约 24 秒后启动：
   - RViz2
   - 多机器人目标分配节点

这套时序是当前仓库稳定启动的重要组成部分，不建议随意压缩延时。

### 6.3 历史拆分式启动

仓库根目录仍保留了若干旧脚本，例如：

- `simulation_env.sh`
- `robot_0_slam.sh`
- `robot_1_slam.sh`
- `robot_2_slam.sh`
- `map_merge.sh`
- `robot_0_fuzzy.sh`
- `robot_1_fuzzy.sh`
- `robot_2_fuzzy.sh`

但当前更推荐直接使用 `start_all.sh`，因为它已经把完整系统启动顺序整合到了一个 launch 中。

---

## 7. 常用话题

### 地图与语义

- `/global_map`：全局融合地图
- `/robot_i/map`：机器人局部地图
- `/semantic_grid_float`：语义值图
- `/semantic_risk_grid_float`：语义风险图

### 前沿与评分

- `/frontiers_info`：前沿扩展信息
- `/frontiers_markers`：前沿可视化 Marker
- `/robot_i/frontier_priority_fuzzy`：机器人对前沿的模糊评分

### 任务与控制

- `/robot_i/task_point`：分配给机器人 `i` 的目标点
- `/exploration_stop`：达到覆盖率阈值后的停止信号

### 位姿与运动

- `/robot_i/odom`
- `/robot_i/cmd_vel`

其中 `i` 默认为 `0, 1, 2`。

---

## 8. 关键配置位置

### 仿真与总启动

- `src/simulation_bringup/launch/full_exploration_system.launch.py`
- `src/simulation_bringup/launch/simulation_bringup.launch.py`
- `src/simulation_bringup/launch/multi_robots_explore.launch.py`

### 机器人初始位姿

- `src/simulation_bringup/config/robot_initial_poses.yaml`

### RViz

- `src/simulation_bringup/config/fuzzy_decision.rviz`

### 世界文件

- `src/simulation_bringup/world/model/model_semantic.world`

### Nav2 / SLAM 参数

- `src/robot_navigation/params/mapper_params_online_async.yaml`
- `src/robot_navigation/params/nav2_params_mppi_robot_0.yaml`
- `src/robot_navigation/params/nav2_params_mppi_robot_1.yaml`
- `src/robot_navigation/params/nav2_params_mppi_robot_2.yaml`

### 模糊决策

- `src/fuzzy_decision/src/fuzzy_decision_node.cpp`

### 目标分配

- `src/goal_select/src/multi_robot_goal_selector_node.cpp`

### 任务执行导航

- `src/multi_robot_navigation/src/minimal_task_navigator.cpp`

### 前沿检测

- `src/rgbd_to_map/src/frontier_detector.cpp`

### 覆盖率监督

- `src/simulation_bringup/scripts/exploration_supervisor.py`

---

## 9. 输出文件与日志

### 指标输出

默认覆盖率统计结果会写到：

```bash
data/metrics/energy_mission_metrics.csv
```

### 运行日志

根目录已有：

```bash
logs/
```

如果你希望长期保留多次实验结果，建议按实验编号归档 `data/` 与 `logs/`。

---

## 10. 已知特点与注意事项

### 10.1 当前默认是 3 机器人流程

虽然部分 launch 参数表面支持 `robot_count`，但从当前实现看，仍有若干节点使用了固定的三机器人配置或显式写死的机器人列表，例如：

- `goal_select` 默认订阅 `/robot_0`、`/robot_1`、`/robot_2`
- `frontier_detector` 默认参数中也给出了 3 台机器人的 frame / name

因此当前工程更适合作为 **固定三机器人系统** 使用。

### 10.2 当前主流程使用 Oracle 语义，而不是在线 CLIP 推理

如果你的目标是复现实验结果，应优先沿用当前默认流程。

如果你希望改成真实视觉语义链路，需要自行调整 `simulation_bringup` 中的总 launch，把 `oracle_semantic_mapper` 替换为你需要的视觉语义节点。

### 10.3 覆盖率统计依赖 ROI 参数

覆盖率停止条件并不是“整张地图已知比例”，而是 `exploration_supervisor.py` 中定义的 **ROI 矩形区域** 内的已知比例。切换世界文件时，通常需要同步检查以下参数：

- `roi_min_x`
- `roi_min_y`
- `roi_max_x`
- `roi_max_y`

这些参数位于：

- `src/simulation_bringup/launch/multi_robots_explore.launch.py`

### 10.4 启动失败时优先检查环境是否重新 source

每次重新编译后，建议重新执行：

```bash
source install/setup.bash
```

否则常见现象包括：

- launch 找不到包
- 新节点不可执行
- 参数文件更新后未生效

---

## 11. 常见操作

### 11.1 重新全量编译

```bash
bash toBuild.sh
source install/setup.bash
```

### 11.2 启动完整系统

```bash
bash start_all.sh
```

### 11.3 单独查看 RViz

完整流程默认会自动启动 RViz；若你单独调试，也可以在 source 环境后直接运行对应 launch 或 RViz 配置。

---

## 12. 后续维护建议

如果后续继续维护该仓库，建议优先完善以下几项：

1. 把 README 中提到的默认 3 机器人逻辑进一步参数化；
2. 清理旧脚本与统一 launch 的重复入口；
3. 将 Oracle 语义与真实视觉语义流程分成两个明确模式；
4. 统一各节点中的硬编码 topic、frame 和参数文件路径；
5. 为 `data/metrics` 和 `logs` 增加按实验自动分目录功能。

---

## 13. 最小使用流程

```bash
cd SenseSimulation_fix
bash toBuild.sh
source install/setup.bash
bash start_all.sh
```

当覆盖率达到设定阈值后，系统会自动停止探索并写出 CSV 指标。
