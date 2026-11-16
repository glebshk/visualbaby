#!/bin/bash
# Install the aufdeckung core and build the ROS 2 workspace (run inside the dev container).
set -e
echo "Installing aufdeckung core..."
if ! python3 -m pip install -e "aufdeckung[viz]"; then
    # Older pip/setuptools in some image versions cannot do editable installs (PEP 660).
    # Put the source folder on Python's path directly instead; edits still apply immediately.
    echo "Editable install not supported here: linking aufdeckung/src into site-packages instead."
    python3 -m pip install "numpy>=1.24" "pyyaml>=6" "matplotlib>=3.7"
    SITE=$(python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")
    mkdir -p "$SITE"
    echo "$PWD/aufdeckung/src" > "$SITE/aufdeckung-src.pth"
fi
python3 -c "import aufdeckung.config" || { echo "aufdeckung is not importable"; exit 1; }

echo "Building ROS workspace..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "Sourcing ROS workspace..."
source install/setup.bash
