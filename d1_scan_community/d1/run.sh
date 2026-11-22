#!/usr/bin/env bash
set -eo pipefail
export D1_TASK_MONOTONIC=$(python3 -c "import time; print(time.monotonic())")
export OPENBLAS_NUM_THREADS=1
cd /workspace/d1_scan_community
exec 8>"/workspace/navigation_ws/d1-planning-preview/single.lock"
flock -n 8 || { echo "RuntimeError: 另一个单点任务仍在运行"; exit 1; }
log_dir=$(mktemp -d "$PWD/task-$(date +%Y%m%d-%H%M%S)-XXXX")
exec > >(tee "$log_dir/task.log") 2>&1
echo "Task arguments: $*"
source /opt/ros/humble/setup.bash
source /workspace/fastlio_ws/install/local_setup.bash
source "$PWD/install/local_setup.bash"
export PYTHONPATH="/workspace/navigation_ws/d1-planning-preview/native-build:/workspace/d1_localization/vendor:${PYTHONPATH}"
export ROS_DOMAIN_ID="${D1_LOCALIZATION_DOMAIN:-87}" ROS_LOCALHOST_ONLY=1
config="$PWD/src/planner/plan_manage/config"
bin="$PWD/install/scan_planner/lib/scan_planner"
# FAST-LIO already registers cloud_registered in odom. Use its LiDAR pose as the ray origin.
"$bin/scan_planner_node" --ros-args --params-file "$PWD/d1/planner.yaml" \
  -r __ns:=/d1_community -p fsm.navi_mode:=3 -p grid_map.frame_id:=odom \
  -p grid_map.cloud_is_world:=true -p grid_map.need_extrinsic:=false \
  -p grid_map.double_cylinder_radius:=0.48 -p grid_map.double_cylinder_offset:=0.0 \
  -p manager.max_vel:=0.12 -p optimization.max_vel:=0.12 \
  -r body_pose:=/fast_lio/odometry_base -r sensor_pose:=/fast_lio/odometry_lidar \
  -r cloud:=/fast_lio/cloud_registered -r planning/bspline:=/d1_community/bspline \
  >"$log_dir/planner.log" 2>&1 &
planner=$!
"$bin/closed_loop_controller" --ros-args --params-file "$PWD/d1/controllers.yaml" \
  -r __ns:=/d1_community -p max_vx:=0.12 -p max_vy:=0.12 -p max_vyaw:=0.25 \
  -r body_pose:=/fast_lio/odometry_base -r planning/bspline:=/d1_community/bspline \
  -r cmd_vel:=/d1_community/candidate >"$log_dir/adapter.log" 2>&1 &
adapter=$!
cleanup() {
  kill -TERM "$planner" "$adapter" 2>/dev/null || true
  for _ in {1..20}; do
    if ! kill -0 "$planner" 2>/dev/null && ! kill -0 "$adapter" 2>/dev/null; then break; fi
    sleep .1
  done
  kill -KILL "$planner" "$adapter" 2>/dev/null || true
  wait "$planner" 2>/dev/null || true
  wait "$adapter" 2>/dev/null || true
}
trap cleanup EXIT
D1_SCAN_PID="$planner" python3 -u d1/trial.py "$@"
