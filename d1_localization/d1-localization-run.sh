#!/usr/bin/env bash
set -eo pipefail
cd /workspace/d1_localization
exec 9>/workspace/d1_localization/run.lock
if ! flock -n 9 || (exec 8<>/dev/tcp/127.0.0.1/8871) 2>/dev/null; then
  echo '定位服务已运行。请只建立 SSH 隧道：ssh -N -L 8871:127.0.0.1:8871 dd@192.168.124.100' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source /workspace/livox_ws/install/local_setup.bash
source /workspace/fastlio_ws/install/local_setup.bash
export LD_LIBRARY_PATH="/workspace/livox_ws/sdk_install/lib:${LD_LIBRARY_PATH}"
export PYTHONPATH="/workspace/d1_localization/vendor:${PYTHONPATH}"
export ROS_DOMAIN_ID="${D1_LOCALIZATION_DOMAIN:-87}" ROS_LOCALHOST_ONLY=1
base_height="${D1_BASE_HEIGHT_M:-0.06}"
processes=()
cleanup() {
  for process in "${processes[@]}"; do kill -TERM -- "-$process" 2>/dev/null || true; done
  for _ in {1..25}; do
    local alive=0
    for process in "${processes[@]}"; do
      if kill -0 -- "-$process" 2>/dev/null; then alive=1; fi
    done
    if (( ! alive )); then break; fi
    sleep .1
  done
  for process in "${processes[@]}"; do kill -KILL -- "-$process" 2>/dev/null || true; done
  for process in "${processes[@]}"; do wait "$process" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
if [[ "${1:-live}" == live ]]; then
  if [[ "${D1_WEB_CONTROL:-0}" == 1 ]]; then
    export PYTHONPATH="/workspace/agibot_d1_sdk/lib/zsl-1/aarch64:${PYTHONPATH}"
    setsid python3 -u /workspace/navigation_ws/d1-velocity-bridge.py --hardware > control.log 2>&1 9>&- &
    processes+=("$!")
  fi
  setsid ros2 launch livox_ros_driver2 msg_MID360s_launch.py > lidar.log 2>&1 9>&- &
  processes+=("$!")
elif [[ "$1" != replay ]]; then
  exit 2
fi
setsid ros2 launch d1_odom_bringup odometry.launch.py > odometry.log 2>&1 9>&- &
processes+=("$!")
setsid python3 -u map_localizer --ros-args -p map_path:=/workspace/maps/real_final/source.pcd \
  -p base_frame:=base_link -p lidar_frame:=lidar_link -p base_height:="$base_height" \
  -p imu_topic:=/livox/imu -p imu_acceleration_scale:=9.80665 -p imu_frame_override:=lidar_imu_link \
  -p xy_radius:=1.0 -p yaw_radius:=30.0 > localizer.log 2>&1 9>&- &
processes+=("$!")
setsid python3 -u d1-localization-web.py /workspace/maps/real_final/map.yaml > web.log 2>&1 9>&- &
processes+=("$!")
if wait -n "${processes[@]}"; then
  result=0
else
  result=$?
fi
echo '定位组件已退出，请查看 /workspace/d1_localization 下的日志。' >&2
for log in lidar.log odometry.log localizer.log web.log control.log; do
  if [[ -f "$log" ]]; then tail -n 4 "$log" >&2; fi
done
exit "$result"
