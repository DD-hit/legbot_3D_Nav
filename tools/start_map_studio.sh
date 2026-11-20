#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source tools/env_humble.sh
export CUDA_PATH="${CUDA_PATH:-/usr/local/cuda-12.8}"
export CUPY_COMPILE_WITH_PTX=1
export LD_LIBRARY_PATH="$CUDA_PATH/lib64:$LD_LIBRARY_PATH"
exec "$PCT_PYTHON" -u tools/map_studio.py "$@"
