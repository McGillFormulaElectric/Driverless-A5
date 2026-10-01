FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-yaml \
      ros-humble-foxglove-bridge \
      ros-humble-ackermann-msgs \
      iproute2 \
      iputils-ping \
      curl \
      git \
      vim \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy pyyaml

ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

WORKDIR /workspace
