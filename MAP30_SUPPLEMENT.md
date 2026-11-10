# 30×30：MRE 两次、原 RACE 一次

入口 `run_map30_mre_race.py`，默认顺序为 Bandwidth-MRE、Bandwidth-MRE、RACE，各机器人容量均显式设置为 0.40 Ah，地图参数为 default。

## 当前仓库即可运行

仓库已包含 MRE 的 SH、三张地图的 launch、推理模块、模型以及原 RACE 模型。默认直接调用当前仓库，不需要 ~/energy_mission 或其他外部工作空间。

本次增加了 ROS Python 节点入口与 launch 文件，更新后需要重新编译一次：

```bash
cd ~/RCAE_IPPO
git pull origin main
conda activate robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**只补两次 MRE，不再补 RACE：**

```bash
python3 run_map30_mre_race.py --race-runs 0 --dry-run
python3 -u run_map30_mre_race.py --race-runs 0
```

**按原计划跑两次 MRE 和一次 RACE：**

```bash
python3 run_map30_mre_race.py --dry-run
python3 -u run_map30_mre_race.py
```

当前仓库模式不要再传 --experiment-root ~/energy_mission；该参数只用于显式选择旧的外部目录布局。

## 单次 SH 命令

MRE：

```bash
bash start_all_bandwidth_mre.sh --map default \
  robot0_battery_capacity_ah:=0.40 \
  robot1_battery_capacity_ah:=0.40 \
  robot2_battery_capacity_ah:=0.40
```

RACE：

```bash
bash start_all_mappo.sh --map default \
  robot0_battery_capacity_ah:=0.40 \
  robot1_battery_capacity_ah:=0.40 \
  robot2_battery_capacity_ah:=0.40
```

默认模型：

- MRE：models/inference/bandwidth_mre/sync_gain_topk_h1_80000/models/best_model.pt
- 原 RACE：models/inference/mappo/energy_distance_safe_seed42/models/best_model.pt

MRE 推理及模型文件复制自原 Bandwidth-MRE 工作空间，provenance.json 保存 SHA-256。新增模型没有重新训练。MRE 直接输出导航目标，不经过 MAPPO 权重节点和 fuzzy goal selector。公共地图、资源、导航和任务监督模块复用本仓库现有实现，不覆盖 RACE/IPPO 的公共源码。

RACE 的 provenance.json 引用九份旧结果配置；原训练汇总记录完成 120000 步，最佳 checkpoint 选于 36476 步。

## 结果与停止

批量结果：

```text
output/map30_mre_race_batches/<批次时间戳>/
  batch_summary.json
  bandwidth_mre_01/attempt_01/console.log
  bandwidth_mre_01/attempt_01/results/map_default/bandwidth_mre/<运行时间戳>/...
  bandwidth_mre_02/attempt_01/...
  race_01/attempt_01/...                 # --race-runs 0 时没有此项
```

任务结束后清理本轮进程并等待 30 秒。电量耗尽等正常任务结局保留并计入；零地图启动异常、启动失败、缺失汇总或超时会停止批次，并保留原始数据。

中断后继续，必须使用与该批次相同的计划参数：

```bash
python3 -u run_map30_mre_race.py --race-runs 0 \
  --resume-dir output/map30_mre_race_batches/<批次时间戳>
```

不带 --resume-dir 再执行会创建新批次、重新执行所选次数，不会自动扫描历史次数。

## 旧外部工作空间模式

只有实验机确实有这两个工作空间时才使用：

```bash
python3 run_map30_mre_race.py --experiment-root ~/energy_mission --dry-run
```

默认外部路径为 compare_with_rl/bandwidth_mre/SenseSimulation_fix 和 main_ablation_greedy/SenseSimulation_fix_v2/SenseSimulation_fix。也可分别用 --mre-workspace、--race-workspace 指定目录，用 --mre-model、--race-model 指定模型。
