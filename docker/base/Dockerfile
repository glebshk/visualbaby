# Basis-Image
FROM ubuntu:22.04

ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------
# 1. Installation of Basic Tools
# ------------------------------------------------------------
WORKDIR /app

COPY /docker/base/ /app/

RUN apt-get update && apt-get install -y jq

RUN apt-get update && \
    jq -r '.apt[]' packages.json | \
    xargs apt-get install -y

# Set locales and timezone
RUN apt-get update && apt-get install -y locales && \
    sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && \
    locale-gen && \
    update-locale LANG=en_US.UTF-8


# ------------------------------------------------------------
# 2. Add ROS 2 and Gazebo Repositories
# ------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list


# Install basic dependencies

RUN apt-get update && apt install -y ros-humble-desktop ros-humble-cv-bridge ros-humble-rclpy-message-converter python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*
 
# ------------------------------------------------------------
# Install Python dependencies
# ------------------------------------------------------------
COPY /docker/base/pip-requirements.txt /app/pip-requirements.txt

RUN pip install -r pip-requirements.txt

# Install Realsense SDK

WORKDIR /app/realsense
RUN apt-get update 

# Clone the repository
RUN git clone https://github.com/IntelRealSense/librealsense.git
WORKDIR /app/realsense/librealsense

# Setup udev rules
RUN cp config/99-realsense-libusb.rules /etc/udev/rules.d/

# Build-Konfiguration
WORKDIR /app/realsense/librealsense/build
RUN cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DFORCE_RSUSB_BACKEND=true \
      -DBUILD_EXAMPLES=false \
      -DBUILD_GRAPHICAL_EXAMPLES=false \
      -DBUILD_NETWORK_DEVICE=OFF \
      -DUSE_SYSTEM_LIBCURL=ON && \
    make -j$(nproc) && \
    make install


# ------------------------------------------------------------
# 4. Setup ROS 2 workspace
# ------------------------------------------------------------
WORKDIR /workspaces/IMAV/
COPY /src/ /workspaces/IMAV/src/
RUN git config --global --add safe.directory /workspaces/IMAV/build/camera_node/_deps/grabthecam-src


# ------------------------------------------------------------
# 4. Source ROS 2 and build workspace
# ------------------------------------------------------------
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspaces/IMAV/

CMD ["/bin/bash"]

