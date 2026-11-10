# 30×30 迷宫：MRE 两次、原 RACE 一次

入口：`run_map30_mre_race.py`。默认顺序为 **Bandwidth-MRE → Bandwidth-MRE → RACE**。
每台机器人电池容量显式设置为 **0.40 Ah**，地图参数为 `default`。

脚本放在 RCAE_IPPO 仓库中，调用实验机已有的两个独立工作空间；不使用 IPPO 模型代替 RACE。
无需因为新增这个 Python 脚本重新编译，但被调用的两个工作空间必须已正确编译、可以单独运行。

## 实验机运行

先激活原实验使用的 ROS / PyTorch 环境。默认寻找 `~/energy_mission`：

```bash
cd ~/RCAE_IPPO
git pull
python3 run_map30_mre_race.py --experiment-root ~/energy_mission --dry-run
python3 -u run_map30_mre_race.py --experiment-root ~/energy_mission
```

本机原始布局对应：

```bash
python3 run_map30_mre_race.py \
  --experiment-root /home/myc/exploration/fuzzy_total/v2_compare/energy_mission \
  --dry-run
```

默认调用位置与模型：

| 方法 | 相对 `--experiment-root` 的工作空间 | 工作空间中的模型 |
|---|---|---|
| Bandwidth-MRE | `compare_with_rl/bandwidth_mre/SenseSimulation_fix` | `models/inference/bandwidth_mre/sync_gain_topk_h1_80000/models/best_model.pt` |
| 原 RACE | `main_ablation_greedy/SenseSimulation_fix_v2/SenseSimulation_fix` | `models/inference/mappo/energy_distance_safe_seed42/models/best_model.pt` |

两个模型均显式传入启动脚本，避免启动脚本默认选用其他版本。
也可分别指定 `--mre-workspace`、`--race-workspace`、`--mre-model`、`--race-model`。
相对模型路径以相应工作空间为基准。模型和启动文件缺失时立即停止，不会自动换用其他模型。

## 保存与结束

- 每轮用独立的 `RESULT_ROOT` 保存原始结果，并写入 `console.log`。
- 收到任务级 `summary.json` 后等待输出刷新，再清理本轮启动的进程及其子进程。
- 两轮之间等待 30 秒，最后一轮结束后直接退出。
- 电量耗尽、600 秒任务时限结束、无可达前沿等正常任务终止均计入，不根据探索成功与否筛选。
- 零地图启动异常、启动失败、缺失汇总或超时会停止批次，保留输出供检查。
- `--timeout-sec` 默认 1800 秒，是批量程序等待汇总的看门狗，不会改变原实验的任务时限。
- 不修改算法、资源模型、导航参数、原启动文件或历史结果。
- 检测到现有 Gazebo / Nav2 实验时拒绝启动，不会清理无关进程。

默认输出：

```text
output/map30_mre_race_batches/<批次时间戳>/batch_summary.json
output/map30_mre_race_batches/<批次时间戳>/bandwidth_mre_01/attempt_01/
output/map30_mre_race_batches/<批次时间戳>/bandwidth_mre_02/attempt_01/
output/map30_mre_race_batches/<批次时间戳>/race_01/attempt_01/
```

各 attempt 中保留原始任务输出、模型来源及哈希。批次清单记录命令、执行位置、终止原因和指标。
脚本复用已有 IPPO 批量程序的汇总检测与进程清理逻辑，实际运行中会持有同一仓库的批次锁。

## 中断后继续

```bash
python3 -u run_map30_mre_race.py --experiment-root ~/energy_mission \
  --resume-dir output/map30_mre_race_batches/<批次时间戳>
```

已完成且有效的任务不再重复运行。异常 attempt 保留，恢复时使用新的 attempt 目录。
如果中断发生在任务已产出有效汇总后，会恢复该汇总。恢复时必须保留相同的工作空间、模型和计划。
**不带 `--resume-dir` 再次运行会创建新批次，重新执行两次 MRE 和一次 RACE**，不会自行扫描历史结果补齐。

`--mre-runs N`、`--race-runs N` 可调整次数，`--pause-sec` 可调整轮间等待。

## 验证

```bash
python3 tools/test_map30_mre_race.py
python3 tools/test_ippo_batch.py
```

测试使用模拟启动和临时目录，不启动 ROS / Gazebo。`--dry-run` 只核对模型与启动文件并打印命令。

## 仓库内单独启动原 RACE

仓库已附带旧 RACE 结果配置所指向的 `energy_distance_safe_seed42/models/best_model.pt`。
相邻 config.json、summary.json 是原训练记录；provenance.json 保存文件哈希和九份历史结果配置的引用。
原训练汇总记录完成 120000 步，最佳 checkpoint 选于 36476 步；这不是后来中断的 mappo_random_battery_seed42 模型。
历史推理配置没有记录模型哈希，因此证据是其记录路径与当前原训练/ROS 副本的哈希一致性。

在本仓库目录执行，无需原 energy_mission 模型目录：

```bash
git pull origin main
conda activate robot
source /opt/ros/humble/setup.bash
bash start_all_mappo.sh --map default \
  robot0_battery_capacity_ah:=0.40 \
  robot1_battery_capacity_ah:=0.40 \
  robot2_battery_capacity_ah:=0.40
```

此次仅修改外层 SH 的默认 checkpoint 路径并添加模型文件，已编译过本仓库时无需重编译。
`run_map30_mre_race.py` 批量入口仍按前文使用两个外部工作空间；本节是仓库内单次 RACE 入口。
