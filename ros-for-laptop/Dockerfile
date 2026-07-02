FROM ros:melodic-ros-core-bionic

# 1) Install ROS desktop-full + RealSense + RPLIDAR drivers
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      ros-melodic-desktop-full \
      ros-melodic-realsense2-camera \
      ros-melodic-rplidar-ros \
      ros-melodic-navigation \
      ros-melodic-explore-lite \
      ros-melodic-robot-localization \
      ros-melodic-imu-tools \
      ros-melodic-depthimage-to-laserscan \
      mesa-utils \
      nano \
      x11-apps \
      libnss-mdns \
      avahi-utils \
      iputils-ping \
      ros-melodic-joint-state-publisher-gui \
      python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

# 2) NVIDIA runtime hints (must come *after* anything that might install libglvnd)
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-graphics,compute,utility}
LABEL com.nvidia.volumes.needed="nvidia_driver"

# 3) Source ROS
ENTRYPOINT ["/bin/bash","-c","source /opt/ros/melodic/setup.bash && exec \"$@\"","--"]
CMD ["bash"]
