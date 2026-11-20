# 从模拟传感器重建建筑点云

`config/robots/go2.yaml` 描述采集使用的传感器话题、安装位姿、里程计坐标系、尺寸和速度限制。`maps/building2_9/map.yaml` 描述用于引导采集的参考地图及版本、楼层和结构图。现有 Web 任务的地图坐标系为 `map`。

首次采集复用 `Building` 世界。从入口初始化，通过原参考地图导航到上层 `(2, -3) m` 并返回入口 `(-5.5, 6) m`。采集包 `raw` 只含原始点云、IMU、静态外参和仿真时钟。`evaluation` 单独保存真值、在线里程计及任务状态。不要在采集期间打开正在写入的 SQLite 文件；进程退出并写出 `metadata.yaml` 后再检查。

在已编译的 Humble 工作区、使用空闲的仿真分区、ROS 域和 Web 端口执行：

```bash
export LEGBOT_ROS_DOMAIN_ID=114 LEGBOT_GZ_PARTITION=legbot-survey-114
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1
source tools/env_humble.sh
run=/data0/rl/legbot_mapping/new_survey
python3 tools/record_building_survey.py "$run" --port 8794
LEGBOT_REPLAY_DOMAIN=115 LEGBOT_REPLAY_RATE=1 bash tools/replay_survey.sh "$run/raw" "$run/replay"
python3 tools/check_building_survey.py "$run" > "$run/quality.json"
end_s=$(python3 -c 'import json,sys;print(json.load(open(sys.argv[1]))["end_s"])' "$run/quality.json")
python3 tools/export_survey.py "$run/replay/registered" "$run/building_round_trip.pcd" --until-time "$end_s" > "$run/round-trip-cloud.json"
```

采集与回放使用不同的 ROS 域。回放只启动 `FAST-LIO` 和里程计坐标变换，按照录制时钟处理雷达与 IMU，在 `odom` 坐标系合并配准后的扫描，以 `5 cm` 体素导出二进制 XYZ 点云。降低 `LEGBOT_REPLAY_RATE` 可减少较慢机器上的回放积压。重新采集或导出时使用新的输出目录。

质量评估使用出发前的静止段，一次性对齐里程计与仿真真值的平移和偏航。入口闭合误差是返回时里程计相对真值的水平位移误差。楼层和楼梯对比使用同一估计坐标系里的去程、回程扫描；仅对双方至少各有三帧的 `20 cm` 平面格计算表面高度中位数之差。楼梯报告反映同一平面位置的垂向重影，不能单独给出踏步边缘的水平误差。建筑参考几何只用于评估区域划分。

第一轮完整往返的复测中，任务结束后机器人静止，长时间回放出现了米级漂移。检查 `quality.json` 的 `task_phase`、`after_return`、`replay_coverage_s` 和各区域 `p95_absolute_m`。上述导出区间覆盖去程和回程。另存全时段点云供诊断时，使用新的文件名并省略 `--until-time`。
