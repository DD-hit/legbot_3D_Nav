# ROS 2 Humble 多楼层初始定位

在 `map_localized` 模式下，用户先选机器狗所在的楼层、位置和朝向。机器狗站定后，用当前激光雷达点云匹配楼宇的 `building2_9.pcd`。匹配通过才发布 `map → odom` 坐标变换，并从校正后的机身位置开始规划。

## 在仿真中操作

在工作区打开第一个终端，运行以下命令。默认出生点在楼宇入口附近，默认目标位于上层。可在有图形桌面的会话中保留 RViz，也可加 `rviz:=false` 运行无界面测试。

```bash
cd /home/rl/workspace/legbot_3D_Nav
source tools/env_humble.sh
ros2 launch legbot_bringup pct_cross_floor_demo.launch.py \
  navigation_source:=map_localized gui:=false rviz:=true
```

在第二个终端也加载环境。用以下命令查看状态。看到 `WAITING_FOR_INITIAL_POSE` 且机器狗已站稳，就可以给初值。

```bash
cd /home/rl/workspace/legbot_3D_Nav
source tools/env_humble.sh
ros2 topic echo /localization/status --qos-durability transient_local
```

- 在 RViz 的 `map` 固定坐标系中点选 `2D Pose Estimate`（二维位姿估计），在楼宇入口附近按住并拖出朝向。图中楼层结构由 PCT 分层地图生成。定位成功后可在 `Views` 中双击 `Follow GO2` 跟随机器人。
- 也可以从第三个已加载环境的终端发送同一接口。下面的初值为地图坐标 `(-4.4, 6.8)`、地面高度 `0 m`、偏航角 `20°`，适用于默认入口演示。

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: -4.4, y: 6.8, z: 0.0}, orientation: {z: 0.173648, w: 0.984808}}}}'
```

上位机拖点可直接复用这个话题。箭头起点给出 `x、y`，方向给出偏航角 `yaw`。偏航四元数取 `z=sin(yaw/2)`、`w=cos(yaw/2)`，这里的 `yaw` 以弧度计。当前搜索范围使用节点参数 `xy_radius` 和 `yaw_radius`，消息中的 `covariance` 不参与评分。

默认 `initial_z_mode:=floor` 会读取消息中的楼面高度，并加上 `0.32 m` 的站立机身高度。若上位机直接给机身中心高度，可选 `initial_z_mode:=base`。RViz 的二维工具通常给出 `z=0`。上层拖点可在启动时选择 `initial_z_mode:=rviz_floor floor_height:=2.95`，也可以在该模式下用 `ros2 param set /map_localizer floor_height 2.95` 选择楼面，再拖动一次。楼层高度单位为米，取值来自当前地图。

默认目标为 `goal_x:=2.0 goal_y:=-3.0 goal_z:=4.5`。这三个参数使用楼宇地图坐标。PCT 将目标吸附到可通行栅格，并生成带机身高度的执行路线，因此实际路线端点和输入值可能有差别。

已有自己的雷达和里程计时，可以单独运行 `ros2 launch legbot_bringup map_localizer.launch.py map_path:=/absolute/path/building.pcd`。原始地图需含 `x、y、z`，支持 `ascii` 和 `binary` 编码。通过 `cloud_topic`、`odom_frame`、`base_frame`、`lidar_frame` 指定实际话题和坐标系。接入全局导航时，原始点云和 PCT 分层地图需要使用同一个坐标系。

## 从初值到行走

1. `FAST-LIO` 用惯性测量单元（IMU）初始化重力方向，生成以 `odom` 为参考的 `/fast_lio/cloud_registered` 点云和 `odom → base` 里程计变换。仿真用模拟激光雷达。真机需要对齐重力的里程计、正确雷达外参和时间戳。
2. `/map_localizer` 收集发出初值后的新扫描，按相同时间戳查机身和雷达变换。默认用 4 帧匹配、后续 2 帧独立复核。采集期间位移超过 `0.15 m` 或转动超过 `8°` 会继续等待静止窗口。
3. 匹配器在初值周围搜索水平位置和航向，用 `GICP`（广义迭代最近点）细化 `x、y、z` 和偏航角。默认水平搜索半径为 `2.5 m`，偏航范围为 `±45°`，可接受的高度修正为 `±0.50 m`。这些是搜索边界，能否通过还取决于当前位置观察到的结构。
4. 节点检查点云重合率、残差、几何约束和多解，并检查从雷达到回波的射线是否穿过地图表面。默认多候选模式还在附近上下最多 `6 m` 的地图范围内复核其他高度。如果另一高度在匹配帧和留出帧上都同样可信，就报楼层歧义，让用户重新选点。
5. 接受后，`/pct/localized_start` 给出当前地图位姿，PCT 生成 `/pct_path_raw`，任务节点将路线转换为 `odom` 中的 `/pct_path`。SCAN 收到新路线并给出可执行的局部轨迹后，机器狗才开始走。

## 查看结果和重试

| 状态或原因 | 含义与操作 |
| --- | --- |
| `LOCALIZED` | 匹配通过。`pose` 为观测时刻机身的 `[x, y, z, yaw]`，坐标单位为米，`yaw` 单位为弧度。运动中的当前位置用 `ros2 run tf2_ros tf2_echo map base` 查看。 |
| `MAP_OUT_OF_COVERAGE` | 初值超出地图覆盖范围。检查地图坐标、原点和楼层。 |
| `LOW_OVERLAP`、`FREE_SPACE_CONFLICT` | 当前回波与候选处重合不足，或部分射线需要穿墙。检查选层和朝向后重发。 |
| `DEGENERATE_GEOMETRY`、`AMBIGUOUS` | 当前几何约束不足或存在多个可信候选。例如只看到地面、长直墙或高度相似的楼层。可在能看到楼梯口、转角等结构的位置重新给初值。 |
| `AMBIGUOUS_FLOOR` | 其他高度也能解释本次观测。确认楼层，并在能区分楼层的位置重新定位。 |
| `LOST` | 定位节点检测到点云或里程计中断、时钟回退等异常。系统撤销行走许可，传感器恢复后重新给初值。 |

每次重新发布 `/initialpose` 都会撤销上一次定位和任务的行走许可。失败时 `/localization/ready` 为 `false`，具体原因和匹配指标在 `/localization/status` 的 JSON 中。成功后的运动位姿由初始 `map → odom` 和连续 `FAST-LIO` 里程计组合得出。长期运行的地图漂移修正需要持续扫描匹配或重新定位。

## `building2_9` 仿真验收

4 个位置分别采集了仿真激光雷达点云。对每处相同观测注入三组水平位置和航向误差，分别为 `0.5 m / 10°`、`1 m / 20°` 和 `2 m / 40°`。每次回放取 5 帧拟合、另取 2 帧验证。Gazebo 机身真值仅供评测。建筑模型的网格原点在世界坐标中为 `(12.999998, -0.000001, -0.10) m`，已按此变换将真值换算到 PCD 坐标系。

| 采样位置 | 多候选成功数 | 最大水平误差 | 最大高度误差 | 最大航向误差 |
| --- | ---: | ---: | ---: | ---: |
| 楼宇入口 | 3/3 | 1.0 cm | 4.0 cm | 0.05° |
| 下层连廊 | 3/3 | 6.4 cm | 0.5 cm | 0.18° |
| 中层平台 | 3/3 | 4.1 cm | 0.1 cm | 0.08° |
| 上层平台 | 3/3 | 4.6 cm | 2.2 cm | 0.49° |

- 多候选模式通过 `12/12` 组，配准计算耗时 `1.75–3.75 s`。同一批观测只从单个初值执行精配准，通过 `4/12` 组。计时范围仅包含配准计算，在线总等待还包括收集新点云和首次加载地图。
- 在 4 个位置分别把初值高度改为 `−2.95 m`、`−1.48 m`、`+1.48 m` 和 `+2.95 m` 的偏差，`16/16` 组被拒绝。另有 `4/4` 组地图外初值被拒绝。这组观测中的拒绝率不能外推到其他楼宇。
- 完整入口任务先输入地图外位姿，确认路线上报数、非零行走命令数都为零，再输入 `(-4.4, 6.8, 0 m, 20°)` 粗位姿。系统从校正后的起点生成 105 个 PCT 路径点，约 `190 s` 后抵达上层。到点瞬间，相对 PCT 末端的真值水平误差为 `0.47 m`。
- 中层任务在开始行走后重新发布地图外初值，观测到连续 30 帧控制输出为零。重新定位成功后系统生成 46 个新路径点并到达上层，真值水平误差为 `0.52 m`。控制器的 `0.50 m` 到点阈值使用里程计估计；这里报告的是独立真值误差。

这两次自动化导航通过 ROS 话题注入粗位姿。另外用 RViz 的实际拖动工具验证了发出的 `/initialpose` 消息、`map` 坐标系和箭头方向。启动、拒绝、配准、重新规划和到点事件记录在服务器工作区的 `log/loc-eval/` 下。
