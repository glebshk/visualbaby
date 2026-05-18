FROM ghcr.io/aachen-drone-development-initiative/imav-videostream/imav:latest

ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------
# Base tools
# ------------------------------------------------------------
RUN apt update && apt install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    git \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# ROS 2 Humble
# ------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" \
> /etc/apt/sources.list.d/ros2.list

# ------------------------------------------------------------
# Gazebo Harmonic repository
# ------------------------------------------------------------
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
> /etc/apt/sources.list.d/gazebo-stable.list

# ------------------------------------------------------------
# Install ROS + Gazebo Harmonic
# ------------------------------------------------------------
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-ros-gzharmonic \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# rosdep
# ------------------------------------------------------------
RUN rosdep init || true
RUN rosdep update || true

# ------------------------------------------------------------
# Python dependencies for PX4
# ------------------------------------------------------------
RUN pip install --no-cache-dir \
    numpy \
    empy \
    jinja2 \
    toml \
    kconfiglib \
    matplotlib

# ------------------------------------------------------------
# PX4 Autopilot
# ------------------------------------------------------------
WORKDIR /sim

RUN git clone -b v1.16.0 --recursive \
    https://github.com/PX4/PX4-Autopilot.git

WORKDIR /sim/PX4-Autopilot

RUN sed -i 's/sudo //g' Tools/setup/ubuntu.sh && \
    bash Tools/setup/ubuntu.sh --no-nuttx

# ------------------------------------------------------------
# Environment
# ------------------------------------------------------------
ENV GZ_VERSION=harmonic
ENV PX4_SIM_MODEL=gz_x500
ENV PX4_GZ_WORLD=default
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/sim/PX4-Autopilot/build/px4_sitl_default/build_gz_plugins
ENV GZ_SIM_RESOURCE_PATH=/sim/PX4-Autopilot/Tools/simulation/gz/worlds:/sim/PX4-Autopilot/Tools/simulation/gz/models


# ------------------------------------------------------------
# ROS sourcing
# ------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# ------------------------------------------------------------
# DDS / ROS settings
# ------------------------------------------------------------
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ------------------------------------------------------------
# Working directory
# ------------------------------------------------------------
WORKDIR /sim/QGC
RUN wget $(curl -s https://api.github.com/repos/mavlink/qgroundcontrol/releases/latest | grep browser_download_url | grep AppImage | cut -d '"' -f 4) \
    && chmod +x *.AppImage &&\
    ./QGroundControl-x86_64.AppImage --appimage-extract && useradd -m imav

# ------------------------------------------------------------
# Default shell
# ------------------------------------------------------------
CMD ["/bin/bash"]
