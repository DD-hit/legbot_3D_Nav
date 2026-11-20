# 公共导航与平台接入

地图定位、`PCT` 全局规划、`SCAN` 局部规划、速度指令授权和 Web 工作台使用同一个导航入口。`GO2` 仿真入口负责提供机器人模型、传感器、里程计和运动控制。两个入口共享导航代码。

## 运行现有仿真

在工作区根目录运行下列命令。`map_profile` 指向地图工作室导出的 `map.yaml`，省略时使用仓库自带地图。

```bash
source tools/env_humble.sh
ros2 launch legbot_bringup pct_cross_floor_demo.launch.py \
  navigation_source:=map_localized publish_ground_truth:=false \
  web_console:=true gui:=false rviz:=false
```

## 独立运行导航

当传感器、坐标变换和底盘控制由外部进程提供时，直接启动公共导航。平台未就绪、定位未成功时，授权速度输出为零。Web 默认监听回环地址，端口默认为 `8765`。

```bash
ros2 launch legbot_bringup navigation.launch.py \
  robot_profile:=/path/to/robot.yaml map_profile:=/path/to/map.yaml
```

可选的 `sensor_odometry.launch.py` 用同一份 `robot_profile` 启动 `FAST-LIO` 和机身里程计转换节点。使用其他里程计时，由对应节点提供下表中的观测。录包回放使用仿真时钟时，为相关入口统一传入 `use_sim_time:=true`，并提供 `/clock`。

| 配置项 | 类型 | 约定 |
|---|---|---|
| `odometry` | `nav_msgs/Odometry` | `odom_frame` 下的机身位姿，子坐标系为 `base_frame`，速度在机身系表达 |
| `registered_cloud` | `sensor_msgs/PointCloud2` | 已在 `odom_frame` 下配准的点云，保留测量时间戳 |
| `lidar_odometry` | `nav_msgs/Odometry` | 同一里程计坐标系下的雷达位姿，用于局部占据地图 |
| `imu` | `sensor_msgs/Imu` | 带实际传感器坐标系和测量时间戳的惯性数据 |
| `ready_topic` | `std_msgs/Bool` | 底盘周期报告就绪状态，使用可靠、瞬态本地（`transient_local`）服务质量；周期须短于 `ready_timeout`。失效时立即发 `false` |
| `cmd_vel_topic` | `geometry_msgs/Twist` | 导航输出获准的机身速度，前、左、逆时针为正。底盘适配器负责转换厂商指令，并在速度指令超时后自行停车 |

里程计提供者负责发布 `odom_frame → base_frame`，机器人描述或静态变换发布器负责机身与雷达、IMU 的固定变换。地图定位节点提供 `map → odom_frame`。每一条变换应只有一个发布者。`/navigation/requested_velocity` 是规划器给速度看门狗的内部指令，底盘订阅 `cmd_vel_topic`。

## 配置倾斜雷达

当前 `config/robot.yaml` 是 `GO2` 仿真配置。`lidar_mount` 描述雷达在机身安装父坐标系中的位姿，现有 `GO2` 的安装父坐标系为 `trunk`。长度用米，滚转、俯仰和偏航用弧度。整台雷达倾斜时，调整这里的安装角。

`imu_in_lidar.xyz` 和 `imu_in_lidar.rpy` 表示内置 IMU 在雷达坐标系下的位姿。机器人模型直接使用该位姿，`FAST-LIO` 使用其逆变换。若该位姿是 `(R,t)`，则逆变换为 `(Rᵀ, −Rᵀt)`。这能避免分别维护两套方向相反的标定。使用外置机身 IMU 时省略 `imu_in_lidar`，由机器人描述提供真实的 IMU 坐标变换，并在对应的 `fastlio_config` 中提供雷达到所选 IMU 的标定。

机身半径、外形偏置、机身标高、速度上限、最低起步速度和末段最低步速均来自平台配置。离线生成的规划地图已有自己的膨胀和台阶约束，更换机身尺寸或越障能力时需要复核地图包。

## 验证

`tests/check_platform_navigation.py` 在独立的 `ROS_DOMAIN_ID` 中运行，检查倾斜安装、外参求逆、替换输入输出话题、就绪心跳超时和空闲零速度。`tests/check_web_navigation_sim.py --round-trip-only` 在已启动的楼宇仿真中验证定位、到达上层和返回入口。
