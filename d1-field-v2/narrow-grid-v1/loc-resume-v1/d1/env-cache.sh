#!/usr/bin/env bash
# Build once per installed overlay. Inherit per-session ROS settings in the caller.
set -eo pipefail
cd "$(dirname "$(realpath "$0")")/.."
source /opt/ros/humble/setup.bash
source /workspace/fastlio_ws/install/local_setup.bash
source /workspace/d1-footprint.FlpF/install/local_setup.bash
source "$PWD/install/local_setup.bash"
python3 - <<'PY'
import os,shlex
for k in ('AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','PYTHONPATH','PATH','PKG_CONFIG_PATH','ROS_DISTRO','ROS_PYTHON_VERSION','ROS_VERSION'):
    if k not in os.environ:continue
    value=shlex.quote(os.environ[k])
    if k.endswith('PATH'):
        print('export '+k+'='+value+'${'+k+':+:$'+k+'}')
    else:
        print('export '+k+'='+value)
PY
