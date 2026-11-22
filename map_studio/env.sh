source /opt/ros/humble/setup.bash
studio=/workspace/map_studio
export PYTHONPATH="$studio/vendor:$studio/gpu:$studio/app/src/legbot_bringup/scripts:${PYTHONPATH:-}"
export CUDA_PATH="$studio/gpu/nvidia/cuda_runtime"
export LD_LIBRARY_PATH="$studio/gpu/nvidia/cuda_nvrtc/lib:$CUDA_PATH/lib:${LD_LIBRARY_PATH:-}"
export CUPY_COMPILE_WITH_PTX=1 MPLBACKEND=Agg OPENBLAS_NUM_THREADS=1
