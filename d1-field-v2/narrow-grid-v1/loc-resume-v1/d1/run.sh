#!/usr/bin/env bash
set -eo pipefail
export D1_TASK_MONOTONIC=${D1_REQUEST_MONOTONIC:-$(python3 -c "import time; print(time.monotonic())")}
export OPENBLAS_NUM_THREADS=1
cd "$(dirname "$(realpath "$0")")/.."
exec 8>"${D1_REPLAY_LOCK:-/workspace/navigation_ws/d1-planning-preview/single.lock}"
flock -n 8 || { echo "RuntimeError: 另一个单点任务仍在运行"; exit 1; }
log_dir=$(mktemp -d "$PWD/task-$(date +%Y%m%d-%H%M%S)-XXXX")
exec > >(tee "$log_dir/task.log") 2>&1
echo "Task arguments: $*"
if [[ -s "$PWD/d1/runtime.env" ]]; then
  source "$PWD/d1/runtime.env"
else
source /opt/ros/humble/setup.bash
source /workspace/fastlio_ws/install/local_setup.bash
source /workspace/d1-footprint.FlpF/install/local_setup.bash
source "$PWD/install/local_setup.bash"
fi
export PYTHONPATH="/workspace/navigation_ws/d1-planning-preview/native-build:/workspace/d1_localization/vendor:${PYTHONPATH}"
export ROS_DOMAIN_ID="${D1_LOCALIZATION_DOMAIN:-87}" ROS_LOCALHOST_ONLY=1
config="$PWD/src/planner/plan_manage/config"
bin="$PWD/install/scan_planner/lib/scan_planner"
# The planner reserves tracking slack outside the guarded model footprint.
speed_values=$(python3 "${D1_SPEED_SETTINGS_CLI:-/workspace/d1_localization/speed_settings.py}" --axes)
read -r D1_FORWARD_MAX D1_LATERAL_MAX D1_YAW_MAX <<< "$speed_values"
export D1_FORWARD_MAX D1_LATERAL_MAX D1_YAW_MAX
footprint_args=$(python3 - <<'PYCONFIG'
from pathlib import Path
import yaml
params = yaml.safe_load(Path('d1/planner.yaml').read_text())['/d1_community/scan_planner_node']['ros__parameters']
slack = yaml.safe_load(Path('d1/tracking.yaml').read_text())['tracking_margin_m']
if not 0 <= slack <= .25:
    raise ValueError('D1 tracking margin outside supported range')
for field in ('front', 'rear', 'half_width'):
    key = 'grid_map.footprint_' + field
    print('-p', f'{key}:={params[key]+slack:.6f}')
PYCONFIG
)
read -ra footprint <<< "${footprint_args//$'\n'/ }"
# FAST-LIO already registers cloud_registered in odom. Use its LiDAR pose as the ray origin.
"$PWD/../scan_planner_node" --ros-args --params-file "$PWD/d1/planner.yaml" \
  -r __ns:=/d1_community -p fsm.navi_mode:=3 -p grid_map.frame_id:=odom \
  -p grid_map.cloud_is_world:=true -p grid_map.need_extrinsic:=false \
  "${footprint[@]}" \
  -p manager.max_vel:="$D1_FORWARD_MAX" -p optimization.max_vel:="$D1_FORWARD_MAX" \
  -r body_pose:=/fast_lio/odometry_base -r sensor_pose:=/fast_lio/odometry_lidar \
  -r cloud:=/fast_lio/cloud_registered -r planning/bspline:=/d1_community/bspline \
  >"$log_dir/planner.log" 2>&1 &
planner=$!
"$bin/closed_loop_controller" --ros-args --params-file "$PWD/d1/controllers.yaml" \
  -r __ns:=/d1_community -p max_vx:="$D1_FORWARD_MAX" -p max_vy:="$D1_LATERAL_MAX" -p max_vyaw:="$D1_YAW_MAX" \
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
