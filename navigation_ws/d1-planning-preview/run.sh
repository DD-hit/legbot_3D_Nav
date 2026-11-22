#!/usr/bin/env bash
set -eo pipefail
cd /workspace/navigation_ws/d1-planning-preview
source /opt/ros/humble/setup.bash
source /workspace/fastlio_ws/install/local_setup.bash
source /workspace/navigation_ws/install/local_setup.bash
export PYTHONPATH="$PWD/native-build:/workspace/d1_localization/vendor:${PYTHONPATH}"
export ROS_DOMAIN_ID="${D1_LOCALIZATION_DOMAIN:-87}" ROS_LOCALHOST_ONLY=1
setsid ros2 run scan_planner scan_planner_node --ros-args --params-file "$PWD/scan.yaml" \
  -p fsm.navi_mode:=3 -p fsm.cancel_on_empty_path:=true -p fsm.planning_horizon:=3.0 \
  -p fsm.global_reference_mode:=piecewise_linear -p grid_map.frame_id:=odom \
  -p body_pose_topic:=/fast_lio/odometry_base -p grid_map.lidar_horizontal_fov_deg:=360.0 \
  -p grid_map.lidar_clear_outside_fov:=false \
  -r /grid_map/body_pose:=/fast_lio/odometry_base -r /grid_map/sensor_pose:=/fast_lio/odometry_lidar \
  -r /grid_map/cloud:=/fast_lio/cloud_registered -r /initial_path:=/dryrun/initial_path \
  -r /planning/bspline:=/dryrun/bspline >scan-preview.log 2>&1 &
planner=$!
trap 'kill -TERM -- "-$planner" 2>/dev/null || true; wait "$planner" || true' EXIT
sleep 3
python3 -u preview.py "$@"
