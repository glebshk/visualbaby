FROM ghcr.io/aachen-drone-development-initiative/imav-videostream/imav:latest

# colcon
RUN pip install colcon-common-extensions

# ------------------------------------------------------------
# ROS 2 Humble
# ------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" \
> /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update || true

# ------------------------------------------------------------
# Gazebo Fortress (IMPORTANT)
# ------------------------------------------------------------
RUN apt update && apt install -y \
    gz-fortress \
    && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Python tools
# ------------------------------------------------------------
RUN pip install numpy empy jinja2 toml kconfiglib matplotlib

# ------------------------------------------------------------
# PX4 Autopilot
# ------------------------------------------------------------
WORKDIR /sim

RUN git clone -b v1.15.1 --recursive https://github.com/PX4/PX4-Autopilot.git

WORKDIR /sim/PX4-Autopilot

RUN sed -i 's/sudo //g' Tools/setup/ubuntu.sh && \
    sed -i 's/matplotlib[^ ]*/matplotlib>=3.0/g' Tools/setup/requirements.txt && \
    bash Tools/setup/ubuntu.sh --no-nuttx

RUN make px4_sitl_default
# ------------------------------------------------------------
# ROS sourcing
# ------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /sim/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash" >> /root/.bashrc
ENV GZ_SIM_RESOURCE_PATH=/workspaces/imav-videostream/simulation
# IMPORTANT: Gazebo Fortress env
RUN echo "export GZ_VERSION=fortress" >> /root/.bashrc

# ------------------------------------------------------------
# Entry
# ------------------------------------------------------------
CMD ["/bin/bash"]
