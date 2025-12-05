# Basis-Image
FROM ubuntu:22.04

ENV LANG=en_US.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# ------------------------------------------------------------
# 1. Grundsystem vorbereiten
# ------------------------------------------------------------
RUN apt-get update 
WORKDIR /app

COPY /docker/apt-requirements.txt /app/apt-requirements.txt
# Install basic dependencies
RUN apt install -y --no-install-recommends $(cat apt-requirements.txt) &&rm -rf /var/lib/apt/lists/*
 
# Arbeitsverzeichnis
WORKDIR /app

# Kopiere lokalen Repo-Inhalt ins Docker-Image
COPY /docker/ /app/

# Locale setzen (wichtig für ROS 2 Tools)
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8


# ------------------------------------------------------------
# 2. ROS 2 Paketquellen hinzufügen
# ------------------------------------------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list
# ------------------------------------------------------------
# Install Python dependencies
# ------------------------------------------------------------
COPY /docker/pip-requirements.txt /app/pip-requirements.txt

RUN pip install -r pip-requirements.txt

# Install Realsense SDK
# Abhängigkeiten installieren
WORKDIR /app/realsense
RUN apt-get update && apt-get upgrade -y \
    && apt-get install git cmake build-essential \
    libssl-dev libusb-1.0-0-dev pkg-config \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    python3-dev python3-pip -y

# Realsense hohlen
RUN git clone https://github.com/IntelRealSense/librealsense.git
WORKDIR /app/realsense/librealsense

# Udev-Regeln anlegen
RUN cp config/99-realsense-libusb.rules /etc/udev/rules.d/

# Build-Konfiguration
WORKDIR /app/realsense/librealsense/build
RUN cmake .. \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_PYTHON_BINDINGS=bool:true \
    -DPYTHON_EXECUTABLE=$(which python3)

# Builden
RUN make
RUN make install
#RUN udevadm control --reload-rules
#RUN udevadm trigger
# Install Python Bindings
#RUN cp ./Release/pyrealsense2.cpython-310-aarch64-linux-gnu.so /usr/lib/python3/dist-packages/pyrealsense2.so

# ------------------------------------------------------------
# 4. Arbeitsverzeichnis anlegen und Quellcode kopieren
# ------------------------------------------------------------
WORKDIR /workspaces/imav-videostream/
COPY /start_ROS.sh /workspaces/imav-videostream/start_ROS.sh
COPY /src/ /workspaces/imav-videostream/src/
RUN git config --global --add safe.directory /workspaces/imav-videostream/build/camera_node/_deps/grabthecam-src
#RUN chmod +x start.sh

# ------------------------------------------------------------
# 4. ROS Umgebung automatisch laden
# ------------------------------------------------------------
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /workspaces/imav-videostream/

#CMD ["/workspaces/imav-videostream/start.sh"]
CMD ["/bin/bash"]

