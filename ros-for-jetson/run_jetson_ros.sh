#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="my-ros-melodic-full:jetson"

# Optional: your workspace on the Jetson
HOST_WS=$HOME/codin/capstone/slam_ws
mkdir -p "$HOST_WS"

# ROS network. The Jetson is the ROS master. Override via the environment if
# your hostname differs, e.g.:
#   ROS_MASTER_URI=http://jetson.local:11311 ROS_HOSTNAME=jetson.local ./run_jetson_ros.sh
ROS_MASTER_URI="${ROS_MASTER_URI:-http://ahmedski-desktop.local:11311}"
ROS_HOSTNAME="${ROS_HOSTNAME:-ahmedski-desktop.local}"

# Choose GPU flag (Jetson usually uses --runtime=nvidia)
GPU_FLAG="--gpus all"
if ! docker run --help | grep -q -- "--gpus"; then
  GPU_FLAG="--runtime=nvidia"
fi
docker run -it --rm \
  --name ros-melodic-jetson \
  --runtime=nvidia \
  --net=host \
  --privileged \
  -e ROS_MASTER_URI="$ROS_MASTER_URI" \
  -e ROS_HOSTNAME="$ROS_HOSTNAME" \
  -v /run/avahi-daemon:/run/avahi-daemon:ro \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
  -e XAUTHORITY=/root/.Xauthority \
  -v /dev:/dev \
  -v "$HOST_WS:/root/ros_ws:rw" \
  -v "$HOME/rs_cuda_ws:/root/rs_cuda_ws:rw" \
  -v /usr/local/lib:/usr/local/lib:ro \
  -v /usr/local/include:/usr/local/include:ro \
  -v /usr/lib/aarch64-linux-gnu/tegra:/host_tegra:ro \
  -v "$HOME/rs_cuda_ws:/home/ahmedski/rs_cuda_ws:rw" \
  -e LD_LIBRARY_PATH=/usr/local/lib:/host_tegra:/usr/local/cuda/lib64:$LD_LIBRARY_PATH \
  "$IMAGE_NAME"
