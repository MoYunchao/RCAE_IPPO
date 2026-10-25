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

## 两张地图各四次，共八次

先关闭其他正在运行的 ROS/Gazebo 实验，再执行：

```bash
python3 -u run_ippo_8runs.py
```

顺序为 default 第 1–4 次，随后 maze40 第 1–4 次。每台机器人设计容量均为 0.40 Ah，完整沿用上述单次命令。
每轮监测本轮目录中的任务 `summary.json`，有有效终止原因后等待 3 秒落盘，再停止该轮启动的进程及子进程。
清理完成后等待 30 秒启动下一轮，最后一轮结束直接退出。

覆盖率达标、达到任务时限、全员失败、无可达前沿等任务终止原因均保留并计入本轮；不会只选成功结果。
单轮异常退出、1800 秒内没有任务汇总或进程无法清理时，整个批次中止并记录错误，不会继续启动下一轮。
Ctrl+C 会清理当前轮并停止整个批次。超时可用 `--timeout-sec 2400` 修改。
预览八次命令而不启动仿真：`python3 run_ippo_8runs.py --dry-run`。

结果结构：

```text
output/ippo_batches/<批次时间戳>/
  batch_summary.json
  default/run_01/console.log
  default/run_01/results/map_default/ippo_fuzzy/<运行时间戳>/
    summary.json
    energy_mission_metrics.csv
    checkpoint_provenance.json
    ippo_infer/...
  default/run_02/... 至 run_04/...
  maze40/run_01/... 至 run_04/...
```

`batch_summary.json` 记录每轮命令、目录、状态、停止原因、FCR 和 T95；每轮目录还保留推理日志及监督器生成的其他 CSV。
`output/` 被 Git 忽略，需要另行打包传回。

## real 地图四次实验

三个机器人设计容量均为 0.70 Ah，模型与上述实验一致：

```bash
python3 -u run_ippo_real_4runs.py
```

共运行四次，每轮结束并清理进程后等待 30 秒。停止、超时和错误处理与八次脚本共用。
结果保存于 `output/ippo_real_batches/<批次时间戳>/real/run_01/` 至 `run_04/`，
批次根目录保存 `batch_summary.json`。预览：`python3 run_ippo_real_4runs.py --dry-run`。
脚本更新后无需重新编译工作空间。

## 整理时的验证

已核对模型哈希，并用此工作空间的推理网络严格加载模型、验证三机器人四维动作输出。
批量脚本的无 ROS 测试覆盖八次顺序、七次等待、模型相对路径、失败中止、超时、Ctrl+C 清理和单独会话子进程清理：

```bash
python3 tools/test_ippo_batch.py
```
