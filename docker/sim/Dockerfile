FROM ghcr.io/aachen-drone-development-initiative/imav-videostream/imav-drone:latest


COPY /simulation/ /workspace/imav-videostream/simulation/
COPY /scripts/ /workspace/imav-videostream/scripts/
COPY /tmux/ /workspace/imav-videostream/tmux/
# ------------------------------------------------------------
# Install Ignition Gazebo (Garden)
# ------------------------------------------------------------
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y \
    gz-garden 
# ------------------------------------------------------------
# Install ROS Humble Bridge for Gazebo Garden
# ------------------------------------------------------------
RUN apt-get install -y ros-humble-ros-gzgarden

# ------------------------------------------------------------
# PX4 Autopilot v1.15 (stable) — Ignition supported!
# ------------------------------------------------------------
WORKDIR /sim
RUN git clone -b v1.15.1 https://github.com/PX4/PX4-Autopilot.git --recursive



WORKDIR /sim/PX4-Autopilot

# Fake docker env for PX4 setup script
RUN touch /.dockerenv
RUN bash ./Tools/setup/ubuntu.sh --no-nuttx


# Build SITL with Ignition
RUN make px4_sitl_default

# ------------------------------------------------------------
WORKDIR /workspace
CMD ["/bin/bash"]
