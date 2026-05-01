ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ghcr.io/sloretz/ros:${ROS_DISTRO}-desktop-full-2025-12-07
FROM ${BASE_IMAGE} AS base

ARG ROS_DISTRO=humble
ARG WORKSPACE_ROOT="/ros2_amiga_ws"
ARG MACHINE_NAME="xavier""

ENV ROS_DISTRO=${ROS_DISTRO}
ENV WORKSPACE_ROOT=${WORKSPACE_ROOT}
ENV DEBIAN_FRONTEND=noninteractive

# --------------------------------------------------------------------------
# Sistem paketleri
# --------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    git wget vim tmux \
    net-tools netcat-traditional \
    python3-full python3-pip \
    build-essential cmake \
    usbutils \
    # RPLidar
    ros-${ROS_DISTRO}-rplidar-ros \
    # Nav2
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-common \
    # SLAM
    ros-${ROS_DISTRO}-slam-toolbox \
    # Foxglove (web tabanlı görselleştirme)
    ros-${ROS_DISTRO}-foxglove-bridge \
    # TF yardımcıları
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------------------------------
# Python bağımlılıkları (base image'ın venv'ine kur)
# --------------------------------------------------------------------------
COPY requirements.txt /requirements.txt
RUN . /.venv/bin/activate && \
    pip install --upgrade pip && \
    pip install -r /requirements.txt

# --------------------------------------------------------------------------
# rosdep — sadece manifests kopyalanır (cache dostu)
# --------------------------------------------------------------------------
WORKDIR ${WORKSPACE_ROOT}

COPY manifests/ /manifests/
RUN rosdep update && \
    rosdep install --from-paths /manifests --ignore-src -r -y

# --------------------------------------------------------------------------
# Kaynak kod
# --------------------------------------------------------------------------
COPY ros2_amiga_ws/ ${WORKSPACE_ROOT}/src/

# --------------------------------------------------------------------------
# Build
# --------------------------------------------------------------------------
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    . /.venv/bin/activate && \
    colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

# --------------------------------------------------------------------------
# Shell konfig — container'a girince her şey hazır
# --------------------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${WORKSPACE_ROOT}/install/setup.bash" >> /root/.bashrc && \
    echo ". /.venv/bin/activate" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
