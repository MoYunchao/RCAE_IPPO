# IPPO 推理与批量实验

本仓库从 `ipporace/ros2_ws` 整理，保留原工作空间的源码、地图、配置、辅助脚本和构建方式。
`toBuild.sh`、所有 CMakeLists.txt 和推理算法源码均保持原样。未上传训练代码、旧 MAPPO 模型、构建产物和实验结果。

## 环境与首次编译

在安装了 ROS 2 Humble、Gazebo Classic 和原工作空间依赖的 Ubuntu 22.04 机器上运行。
沿用 PACE 实验机环境即可；激活的 Python 环境需要能导入 PyTorch、NumPy 和 ROS Python 包。

```bash
cd ~
git clone git@github.com:MoYunchao/RCAE_IPPO.git
cd RCAE_IPPO
conda activate robot
source /opt/ros/humble/setup.bash
bash toBuild.sh
```

`toBuild.sh` 是原有全量重新构建脚本，会先清理本仓库的 build、log、install。
以后更新用 `git pull origin main`；更新源码后再编译。当前整理机器未安装 ROS，尚未执行完整 ROS 编译和仿真。

## 模型

默认模型：`models/inference/ippo/best_model.pt`。
来自训练运行 `ippo_random_battery_seed42/models/best_model.pt`，与用户指定的模型逐字节一致。
相邻的 `config.json` 保留训练参数，`provenance.json` 记录 SHA-256 和模型结构。
训练配置中原图数据的绝对路径作为记录保留，推理不需要这些离线训练数据。

模型标记为 IPPO，actor 输入 10 维、本地 critic 输入 10 维，输出四个独立 Beta 权重。
推理代码内部沿用 `mappo` 类名、launch 文件名和参数名，以保持原有调用链。

## 单次运行

```bash
bash start_all_ippo.sh --map default \
  models/inference/ippo/best_model.pt \
  robot0_battery_capacity_ah:=0.40 \
  robot1_battery_capacity_ah:=0.40 \
  robot2_battery_capacity_ah:=0.40
```

另一张地图只需把 `default` 改成 `maze40`。模型路径也可省略，默认就是上述仓库内模型。
单次结果位于 `output/ippo_runs/map_<地图>/ippo_fuzzy/<时间戳>/`。

## 补充实验：连同已有结果每张地图五次

文件名沿用 `run_ippo_8runs.py` / `run_ippo_real_4runs.py`，但默认行为已改为补齐五次，不再重新跑八次或四次。

历史基线清单：`configs/ippo_existing_results_20261025.json`，来自 `race_ippo_results.tar.gz`，保存压缩包及各条 summary 的 SHA-256、模型哈希、原始指标和纳入/排除理由。保留所有原始运行记录，不按探索成功率筛选。

| 地图 | 历史任务次数 | 启动异常 | 已有可用次数 | 首次需要补跑 | 每台容量 |
|---|---:|---:|---:|---:|---:|
| default（30×30） | 4 | 1 | 3 | 2 | 0.40 Ah |
| maze40（40×40） | 4 | 2 | 2 | 3 | 0.40 Ah |
| real | 4 | 0 | 4 | 1 | 0.70 Ah |

旧异常对应 default/run_03、maze40/run_02、maze40/run_04：已知地图格数为 0、机器人未移动，TF 不连通。另两次 `python: command not found` 的批次没有任务汇总，不作为探索样本。正常探索后的电池耗尽、任务时限或其他未完成结果均计入五次，绝不只保留成功结果。

容量必须沿用这批实际结果的 0.40/0.70 Ah；此前讨论过的 0.50 Ah 不混入本组。所有补跑使用同一个 IPPO checkpoint，脚本会验证 SHA-256。

在实验机更新脚本后，无需重新编译 ROS 包。确认没有其他 ROS/Gazebo 实验运行，依次执行：

```bash
conda activate robot
source /opt/ros/humble/setup.bash
python3 -u run_ippo_8runs.py
python3 -u run_ippo_real_4runs.py
```

第二条实验命令应在第一条结束后再执行。可先预览：

```bash
python3 run_ippo_8runs.py --dry-run
python3 run_ippo_real_4runs.py --dry-run
```

脚本保留本轮 summary、checkpoint_provenance、console.log、推理和轨迹数据；等待任务结束并清理该轮进程后，再等待 30 秒启动下一轮。若 summary 的 known_cells 为 0，则标记 startup_failed、保留原始数据、不计入五次，每个缺失样本最多尝试三次（可通过 --max-startup-attempts 修改）。连续启动异常或其他运行错误会中止批次，避免无限重试。该判断不能修复 SLAM/TF 根因；如果反复失败，应先修复启动环境。

新增结果目录：

```text
output/ippo_supplement_batches/<批次时间戳>/batch_summary.json
output/ippo_supplement_batches/<批次时间戳>/<地图>/sample_04/attempt_01/...
output/ippo_real_supplement_batches/<批次时间戳>/...
```

每次执行都会合并历史清单与本机 output 下的补跑汇总，按 summary 哈希去重，只补缺少的次数。中断后再次执行原命令即可继续；已形成有效任务汇总、但在清理阶段中断的样本也会恢复计数。已有五次时不再启动仿真。两个脚本共用运行锁，不能同时启动。

跨机器继续补跑时，请同时传递完整的新增批次目录（含 summary 与 provenance），只复制 batch_summary.json 无法核对和恢复计数。历史压缩包和解压原始数据保留在本机、不提交 Git；历史清单会随源码推送。

这轮只调整实验调度与记录，不修改 IPPO 策略、奖励、机器人能耗或 ROS 节点。最终统计保留每张地图五次、同一训练模型的评估粒度；不等同于五个独立训练种子。

## 整理时的验证

已核对模型哈希，并用此工作空间的推理网络严格加载模型、验证三机器人四维动作输出。
随附的无 ROS 检查脚本已随补跑计划调整，包含五次补跑顺序、四次等待、模型相对路径、失败中止、超时、Ctrl+C 清理和单独会话子进程清理。本次仅执行语法与 dry-run 计划检查，未运行该测试脚本，也未启动 ROS 仿真：

```bash
python3 tools/test_ippo_batch.py
```
