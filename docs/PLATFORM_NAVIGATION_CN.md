# 公共导航与平台接入

地图定位、`PCT` 全局规划、`SCAN` 局部规划、速度指令授权和 Web 工作台使用同一个导航入口。`GO2` 仿真入口负责提供机器人模型、传感器、里程计和运动控制。两个入口共享导航代码。

## 运行现有仿真

在工作区根目录运行下列命令。`robot` 选择机型，`map` 选择场地。当前仿真默认使用 `navigation-map.zip` 导出的 `building_edited`，包含一楼至四楼。`map:=building2_9` 可选择五层示例。

```bash
source tools/env_humble.sh
ros2 launch legbot_bringup bringup.launch.py robot:=go2 backend:=sim gui:=false
```

新入口默认启用 Web 和地图定位。仿真雷达与惯性测量单元（`IMU`）提供观测，`FAST-LIO` 提供里程计。需要打开 Gazebo 窗口时使用 `gui:=true`。原有启动命令仍可用。

## 模型与配置

| 需要调整什么 | 工作区内的位置 |
|---|---|
| GO2 机身模型、网格、关节 | `src/robot_description/go2/`，运行入口为 `xacro/robot.xacro` |
| 共用雷达和 IMU 的仿真描述 | `src/robot_description/sensors/livox.xacro` |
| 指定模型、控制策略、雷达外参、话题及机身限制 | `config/robots/go2.yaml` |
| 雷达驱动配置与里程计参数 | `config/sensors/livox/` 和 `config/sensors/fastlio/` |
| 场地地图与各图的导航、仿真设置 | `maps/<地图名>/map.yaml` |
| 共用的局部规划参数 | `config/navigation/` |
| GO2 控制器参数与配套策略权重 | `config/control/go2/` |

配置和模型通过 `legbot_config`、`robot_description` 两个 ROS 资源包安装。在当前 Humble 工作区修改源文件后执行以下命令，再重新启动。单独覆盖配置时可以直接传 `robot_profile:=/绝对路径/go2.yaml`。

```bash
source tools/env_humble.sh
colcon build --base-paths src --build-base build/humble-bringup --install-base install/humble-bringup \
  --packages-select legbot_config robot_description go2_description legbot_bringup
source tools/env_humble.sh
```

`Livox` 厂商启动器从驱动包的安装目录读取设备配置。编辑 `config/sensors/livox/` 后，当前 Humble 工作区还需运行 `colcon build --base-paths src --build-base build/humble-lio --install-base install/humble-lio --packages-select livox_ros_driver2`。

新增机型时把模型放到 `src/robot_description/<机型>/`，在 `config/robots/` 添加同名文件。已有独立驱动和底盘控制的机型可用 `backend:=external` 接入下表的话题和变换。目前 `backend:=sim` 提供 GO2 的 Gazebo 与控制器。

## 管理多张地图

地图工作室导出的地图包解压后，将含 `map.yaml` 的目录放到地图库下，目录名就是启动时的 `map`。在源码工作区执行 `source tools/env_humble.sh` 后，默认地图库为工作区的 `maps/`。也可用 `maps_dir:=/绝对路径` 为本次启动指定地图库，或用 `LEGBOT_MAPS_DIR` 环境变量指定常用位置。新增地图后直接重新启动即可。

每张图的 `pcd` 指定定位点云，`tomogram` 指定规划栅格，`display` 指定 Web 结构图。`floors` 按高度递增列出 `[地面标高, 楼层名]`。相对路径写作 `./文件名`，以这张图的 `map.yaml` 所在目录为准。`id` 区分场地，`version` 区分地图版本，Web 按两者保存常用点位和路线。

统一启动入口支持在地图的 `navigation` 中覆盖 `finish_dist`（到点距离，米）和 `goal_settle_time`（到点稳定时间，秒）。同名启动参数优先。`simulation.world`、`world_name` 和 `spawn` 可指定本场地的 Gazebo 世界、SDF 世界名与出生位姿，`yaw` 采用弧度。出生位姿只用于放置仿真机身。省略世界配置时，`backend:=sim` 沿用演示楼宇世界。真实场地接入外部观测时使用 `backend:=external`。

显式传入 `map_profile:=/绝对路径/map.yaml` 时直接加载该文件。仓库内保留了原地图配置和示例资产路径的兼容入口。

## 独立运行导航

当传感器、坐标变换和底盘控制由外部进程提供时，直接启动公共导航。平台未就绪、定位未成功时，授权速度输出为零。Web 默认监听回环地址，端口默认为 `8765`。

```bash
ros2 launch legbot_bringup bringup.launch.py backend:=external \
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

当前 `config/robots/go2.yaml` 是 `GO2` 仿真配置。`lidar_mount` 描述雷达在机身安装父坐标系中的位姿，现有 `GO2` 的安装父坐标系为 `trunk`。长度用米，滚转、俯仰和偏航用弧度。整台雷达倾斜时，调整这里的安装角。

`imu_in_lidar.xyz` 和 `imu_in_lidar.rpy` 表示内置 IMU 在雷达坐标系下的位姿。机器人模型直接使用该位姿，`FAST-LIO` 使用其逆变换。若该位姿是 `(R,t)`，则逆变换为 `(Rᵀ, −Rᵀt)`。这能避免分别维护两套方向相反的标定。使用外置机身 IMU 时省略 `imu_in_lidar`，由机器人描述提供真实的 IMU 坐标变换，并在对应的 `fastlio_config` 中提供雷达到所选 IMU 的标定。

机身半径、外形偏置、机身标高、速度上限、最低起步速度和末段最低步速均来自平台配置。离线生成的规划地图已有自己的膨胀和台阶约束，更换机身尺寸或越障能力时需要复核地图包。

## 验证

`tests/check_platform_navigation.py` 在独立的 `ROS_DOMAIN_ID` 中运行，检查倾斜安装、外参求逆、替换输入输出话题、就绪心跳超时和空闲零速度。`tests/check_web_navigation_sim.py --round-trip-only` 在已启动的楼宇仿真中验证定位、到达上层和返回入口。
