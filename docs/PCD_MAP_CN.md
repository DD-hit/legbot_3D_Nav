# 从已有 PCD 生成多楼层导航地图

`tools/build_pcd_map.py` 读取重力对齐、单位为米的 `PCD` 和地图配置中的楼层标高、名称、地图编号与坐标系。它使用当前 `SceneBuilding` 通行参数生成 `PCT` 分层代价图，再从原始点云生成楼层结构图。`Building.dae` 仅由独立验收测试读取。

## 转换和运行

### 可视化确认楼层

在下方配置完 Humble 与 CUDA 环境后，可以用同一份原始点云启动确认页面。

```bash
"$PCT_PYTHON" tools/floor_review.py src/pct_planner/src/pcd/building2_9.pcd /data0/rl/legbot_mapping/floor-review --infer-isolated
```

在服务器上打开 `http://127.0.0.1:8798/`。从开发机访问时使用 `ssh -NT -L 8798:127.0.0.1:8798 rl@100.73.76.90` 后在开发机打开同一地址。导入其他建筑时将命令的第一个路径换为对应的 PCD。支持 ASCII 与未压缩二进制 PCD，输入坐标须以米为单位，Z 轴向上。

1. 程序用点云的 XYZ 坐标统计各高度的水平覆盖，再按局部上方空间与连续平面大小筛选。初次导入所有候选均待勾选。屋顶、桌面或局部高台也可能入选，狭小或稀疏的平台可能漏检。
2. 点「预览」对照建筑侧视和水平切片。侧视可切换观察方向，俯视用绿色显示标高上下 3.5 cm 范围的采样。此宽度只用于预览。滑动标高可以检查漏检平台，再点「添加当前预览高度」。
3. 勾选用于导航的楼层和楼梯中间平台，填写名称。通常保留实测标高即可。楼层标高用于分层与楼梯端点估计，选点的地面高度由对应栅格提供。可视化流程按点云最低高度对齐 PCT 切片基准，改名与微调平台标高均不会移动整个规划切片网格。
4. 点「确认并生成地图」。每次生成独立目录，成功后显示 `map.yaml` 路径。同一工作目录、源点云与导入选项下，再打开页面可恢复上次成功确认的楼层。启动导航时将生成的 `map.yaml` 传给 `map_profile`。

### 使用已有配置直接转换

在 Humble 工作区执行，输出目录使用一个新路径。下面的 CUDA 环境适用于当前服务器的 RTX 5090。

```bash
export LEGBOT_ROS_DOMAIN_ID=118 LEGBOT_GZ_PARTITION=building-pcd-118
source tools/env_humble.sh
export CUDA_PATH=/usr/local/cuda-12.8
export LD_LIBRARY_PATH="$CUDA_PATH/lib64:$LD_LIBRARY_PATH"
export CUPY_COMPILE_WITH_PTX=1
bundle=/data0/rl/legbot_mapping/new_building_bundle
"$PCT_PYTHON" tools/build_pcd_map.py src/pct_planner/src/pcd/building2_9.pcd "$bundle" \
  --profile src/legbot_bringup/config/map.yaml --infer-isolated
PYTHONPATH="src/legbot_bringup/scripts:$PYTHONPATH" "$PCT_PYTHON" tests/check_building_pcd_bundle.py "$bundle"
ros2 launch legbot_bringup pct_cross_floor_demo.launch.py map_profile:="$bundle/map.yaml" \
  navigation_source:=map_localized web_console:=true web_port:=8797 gui:=false rviz:=false
```

测试入口 `python3 tests/check_web_navigation_sim.py --url http://127.0.0.1:8797 --round-trip-only` 要求从入口新启动且尚未定位的仿真实例。该测试通过 Web 接口给定初始位姿，提交上层平台、入口两个带朝向的目标点，并检查任务系统是否依序确认两点。

## 查看产物

- `source.pcd` 是原始点云的副本，供定位匹配。`planning.pickle` 是规划器和网页选点共用的分层地图。
- `display.json` 包含墙体投影、楼面图像、楼梯标记和各楼层可选栅格的校验值。`map.yaml` 的相对路径使整包可以搬移。
- `report.json` 记录输入点云的 SHA-256、推断候选坐标、楼梯几何、各层可选格数以及本次是否启用补点。

## 推断范围

默认保留观测缺口，仅报告候选。显式添加 `--infer-isolated` 后，对每层平坦平台内的单格缺测进行插值。以 `10 cm` 分辨率为例，候选须有周围 `5 × 5` 范围内其他 `24` 个栅格的楼面观测、靠近中心四个象限的实测点，并通过局部高差、边缘和净空筛查。通过的样本只参与规划地图重算。点云稀疏、地面倾斜或缺口较大时可能保留受限区。单份点云无法证明遮挡后一定存在地面，真机地图可先用默认模式核查候选。

楼梯检测在相邻标高之间提取连通的可行走区域，寻找连续的踏步高度组并拟合上升方向。输出描述可通行中心带、估计踏步数和上下平台。网页在同一平面位置有多段候选时保留楼梯选择。当前方法面向直梯，转折处需要有单独标注的中间平台；螺旋梯、连续斜坡和点云覆盖不足的楼梯仍需专门验证。楼层标高沿用输入配置，屋顶是否有路径以规划结果为准。
