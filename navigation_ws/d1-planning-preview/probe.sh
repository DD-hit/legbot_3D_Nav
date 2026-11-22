#!/usr/bin/env bash
set -eo pipefail
cd /workspace/navigation_ws/d1-planning-preview
exec 8>"$PWD/single.lock"
flock -n 8 || { echo "RuntimeError: 另一个单点任务仍在运行"; exit 1; }
log_dir=$(mktemp -d "$PWD/task-$(date +%Y%m%d-%H%M%S)-XXXX")
exec > >(tee "$log_dir/task.log") 2>&1
echo "Task arguments: $*"
source /opt/ros/humble/setup.bash
source /workspace/fastlio_ws/install/local_setup.bash
source /workspace/navigation_ws/install/local_setup.bash
export PYTHONPATH="$PWD/native-build:/workspace/d1_localization/vendor:${PYTHONPATH}"
export ROS_DOMAIN_ID="${D1_LOCALIZATION_DOMAIN:-87}" ROS_LOCALHOST_ONLY=1
/workspace/navigation_ws/install/scan_planner/lib/scan_planner/scan_planner_node --ros-args --params-file "$PWD/scan.yaml" \
  -p fsm.navi_mode:=3 -p fsm.cancel_on_empty_path:=true -p fsm.planning_horizon:=3.0 \
  -p fsm.global_reference_mode:=piecewise_linear -p grid_map.frame_id:=odom \
  -p body_pose_topic:=/fast_lio/odometry_base -p grid_map.lidar_horizontal_fov_deg:=360.0 \
  -p grid_map.lidar_clear_outside_fov:=false \
  -r /grid_map/body_pose:=/fast_lio/odometry_base -r /grid_map/sensor_pose:=/fast_lio/odometry_lidar \
  -r /grid_map/cloud:=/fast_lio/cloud_registered -r /initial_path:=/d1_trial/initial_path \
  -r /planning/bspline:=/d1_trial/bspline >"$log_dir/planner.log" 2>&1 &
planner=$!
/workspace/navigation_ws/install/scan_planner/lib/scan_planner/scan_go2_cmd_adapter --ros-args --params-file "$PWD/scan.yaml" \
  -p body_pose_topic:=/fast_lio/odometry_base -p require_route_stamp:=true \
  -p respect_goal_heading:=true -p finish_dist:=0.12 -p goal_settle_time:=0.45 \
  -p max_vx:=0.12 -p max_vy:=0.12 -p max_vyaw:=0.25 \
  -r /cmd_vel:=/d1_trial/candidate -r /scan/planning/bspline:=/d1_trial/bspline >"$log_dir/adapter.log" 2>&1 &
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
sleep 2
python3 -u shadow_probe.py "$@"
