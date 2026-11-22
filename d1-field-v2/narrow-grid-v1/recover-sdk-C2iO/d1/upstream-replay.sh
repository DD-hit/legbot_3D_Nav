#!/bin/bash
set -eo pipefail
ws=/workspace/d1_scan_community
source /opt/ros/humble/setup.bash
source "$ws/install/local_setup.bash"
for mode in open static crossing; do
  case $mode in open) id=196;; static) id=197;; crossing) id=198;; esac
  (
    cd "$ws/upstream-$mode"
    export ROS_DOMAIN_ID=$id ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH="/workspace/d1_localization/vendor:${PYTHONPATH:-}"
    bin="$ws/install/scan_planner/lib/scan_planner"
    "$bin/scan_planner_node" --ros-args --params-file "$ws/d1/planner.yaml" -r __ns:=/d1_community -p fsm.navi_mode:=3 -p grid_map.frame_id:=odom -r body_pose:=/fast_lio/odometry_base -r sensor_pose:=/fast_lio/odometry_lidar -r cloud:=/fast_lio/cloud_registered -r planning/bspline:=/d1_community/bspline >planner.log 2>&1 & pl=$!
    "$bin/closed_loop_controller" --ros-args --params-file "$ws/d1/controllers.yaml" -r __ns:=/d1_community -r body_pose:=/fast_lio/odometry_base -r planning/bspline:=/d1_community/bspline -r cmd_vel:=/d1_community/candidate >ctrl.log 2>&1 & ctrl=$!
    python3 test.py loop >motion.log 2>&1 & fx=$!
    trap 'kill "$pl" "$ctrl" "$fx" 2>/dev/null || true; wait || true' EXIT
    python3 "$ws/d1/reference_replay.py" >result.log 2>&1
  )
done
