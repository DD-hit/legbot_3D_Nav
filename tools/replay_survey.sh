#!/usr/bin/env bash
set -e -o pipefail
if [[ $# != 2 ]]; then
  echo 'Usage: replay_survey.sh RAW_BAG NEW_OUTPUT_DIRECTORY' >&2
  exit 2
fi
workspace="$(cd "$(dirname "$0")/.." && pwd)"
raw_bag="$(realpath "$1")"
mkdir "$2"
result="$(realpath "$2")"
export LEGBOT_ROS_DOMAIN_ID="${LEGBOT_REPLAY_DOMAIN:-113}"
source "$workspace/tools/env_humble.sh"
cd "$result"
profile="${LEGBOT_ROBOT_PROFILE:-$workspace/src/legbot_bringup/config/robot.yaml}"
profile_arg() { python3 -c 'import sys,yaml;print(yaml.safe_load(open(sys.argv[1]))[sys.argv[2]])' "$profile" "$1"; }
cloud="$(profile_arg raw_cloud)"
imu="$(profile_arg imu)"
registered="$(profile_arg registered_cloud)"
base_odom="$(profile_arg odometry)"
lio_config="$(profile_arg fastlio_config)"
setsid ros2 launch legbot_bringup fastlio.launch.py use_sim_time:=true align_gravity:=true sensor_queue_depth:=1024 \
  config:="$workspace/src/legbot_bringup/config/$lio_config" sensor_frame:="$(profile_arg sensor_frame)" \
  base_frame:="$(profile_arg base_frame)" odom_frame:="$(profile_arg odom_frame)" raw_cloud_topic:="$cloud" imu_topic:="$imu" \
  registered_cloud_topic:="$registered" base_odom_topic:="$base_odom" lidar_odom_topic:="$(profile_arg lidar_odometry)" \
  lidar_frame:="$(profile_arg lidar_frame)" >mapping.log 2>&1 &
mapper=$!
setsid ros2 bag record --use-sim-time -o registered "$registered" "$base_odom" >record.log 2>&1 &
recorder=$!
stop_group() {
  kill -INT -- "-$1" 2>/dev/null || true
  wait "$1" 2>/dev/null || true
}
trap 'stop_group "$recorder"; stop_group "$mapper"' EXIT
sleep 3
ros2 bag play "$raw_bag" --disable-keyboard-controls --rate "${LEGBOT_REPLAY_RATE:-1}" --delay 2 \
  --topics "$cloud" "$imu" /tf_static /clock >play.log 2>&1
sleep 2
