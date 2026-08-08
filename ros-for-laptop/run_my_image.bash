#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="my-ros-melodic-full"

HOST_WS=$HOME/coding/ros-stuff/slam_ws
mkdir -p "$HOST_WS"

# ROS network. The master runs on the Jetson; the laptop publishes/subscribes
# using its own mDNS hostname. Override either via the environment, e.g.:
#   ROS_MASTER_URI=http://jetson.local:11311 ROS_HOSTNAME=laptop.local ./run_my_image.bash
ROS_MASTER_URI="${ROS_MASTER_URI:-http://ahmedski-desktop.local:11311}"
ROS_HOSTNAME="${ROS_HOSTNAME:-sahme-ROG-Zephyrus-G14-GA401IU-GA401IU.local}"

XAUTH=/tmp/.docker.xauth
if [ ! -f "$XAUTH" ]; then
  xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
  if [ -n "$xauth_list" ]; then
    echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
  else
    touch "$XAUTH"
  fi
  chmod a+r "$XAUTH"
fi

docker run -it --rm \
  --name ros-melodic-laptop \
  --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=$XAUTH" \
  --env="ROS_MASTER_URI=$ROS_MASTER_URI" \
  --env="ROS_HOSTNAME=$ROS_HOSTNAME" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTH:$XAUTH" \
  --volume="$HOST_WS:/root/ros_ws:rw" \
  --volume="/run/avahi-daemon:/run/avahi-daemon:ro" \
  --runtime=nvidia \
  "$IMAGE_NAME" \
  bash
