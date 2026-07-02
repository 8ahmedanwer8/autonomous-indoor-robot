FROM ros:melodic-ros-core-bionic

SHELL ["/bin/bash", "-c"]

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
      ros-melodic-desktop-full \
      ros-melodic-realsense2-camera \
      ros-melodic-rplidar-ros \
      ros-melodic-rtabmap-ros \
      ros-melodic-robot-localization \
      ros-melodic-imu-tools \
      ros-melodic-navigation \
      ros-melodic-teleop-twist-keyboard \
      ros-melodic-joint-state-publisher-gui \
      mesa-utils \
      nano \
      vim \
      x11-apps \
      libnss-mdns \
      avahi-utils \
      iputils-ping \
      python-rosdep \
      python-rosinstall \
      python-rosinstall-generator \
      python-wstool \
      build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update || true

ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-graphics,compute,utility}
LABEL com.nvidia.volumes.needed="nvidia_driver"

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

WORKDIR /root/ros_ws

ENTRYPOINT ["/bin/bash","-c","source /opt/ros/melodic/setup.bash && exec \"$@\"","--"]
CMD ["bash"]
