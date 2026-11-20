#!/usr/bin/env bash

legbot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${LEGBOT_GZ_OVERLAY:-${HOME}/workspace/gz_harmonic_ws/install}/local_setup.bash"

for legbot_overlay in humble humble-gazebo humble-pct humble-lio humble-controller humble-bringup; do
  legbot_setup="${legbot_root}/install/${legbot_overlay}/local_setup.bash"
  if [[ -f "${legbot_setup}" ]]; then
    source "${legbot_setup}"
  fi
done

export GZ_CONFIG_PATH="/usr/local/share/gz:/usr/share/gz${GZ_CONFIG_PATH:+:${GZ_CONFIG_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${legbot_root}/install/humble-gazebo/gz_quadruped_hardware/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
export PCT_PYTHON="${legbot_root}/.venv-humble/bin/python3"
export PCT_THIRDPARTY_LIB_DIR="/opt/ros/humble/lib/x86_64-linux-gnu"
legbot_torch_dir="${LEGBOT_TORCH_ROOT:-${legbot_root}/third_party/libtorch}"
export LD_LIBRARY_PATH="${legbot_root}/third_party/onnxruntime/lib:${legbot_torch_dir}/lib:${legbot_root}/third_party/Livox-SDK2/install/lib:${PCT_THIRDPARTY_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export ROS_DOMAIN_ID="${LEGBOT_ROS_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY=1
export LEGBOT_MAPS_DIR="${LEGBOT_MAPS_DIR:-${legbot_root}/maps}"
export GZ_PARTITION="${LEGBOT_GZ_PARTITION:-legbot-humble-${USER}}"
export GZ_IP="${LEGBOT_GZ_IP:-127.0.0.1}"

unset legbot_root legbot_overlay legbot_setup legbot_torch_dir
