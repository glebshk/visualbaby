#!/bin/bash
# Install the aufdeckung core and build the ROS 2 workspace (run inside the dev container).
set -e
# pip notices that are expected inside this root-only container, not problems:
export PIP_ROOT_USER_ACTION=ignore PIP_DISABLE_PIP_VERSION_CHECK=1
LOG=/tmp/aufdeckung-pip.log
echo "Installing aufdeckung core..."
if ! python3 -m pip install -q -e "aufdeckung[viz]" >"$LOG" 2>&1; then
    # Older pip/setuptools in some image versions cannot do editable installs (PEP 660).
    # Put the source folder on Python's path directly instead; edits still apply immediately.
    echo "  (editable install not supported by this image's pip: linking aufdeckung/src instead, see $LOG)"
    # "Can't uninstall ... outside environment" below is fine: the new version goes to
    # /usr/local, which Python searches before Ubuntu's own packages.
    python3 -m pip install -q "numpy>=1.24" "pyyaml>=6" "matplotlib>=3.7"
    SITE=$(python3 -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")
    mkdir -p "$SITE"
    echo "$PWD/aufdeckung/src" > "$SITE/aufdeckung-src.pth"
fi
python3 -c "import aufdeckung.config" || { echo "aufdeckung is not importable"; exit 1; }

# The devcontainers write /root/.ros/fastdds.xml with an empty <userTransports/>, which this Fast DDS
# rejects: every ROS program then prints four "XMLPARSER Error" lines and falls back to defaults.
# Same intent (built-in transports), valid XML:
if [ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]; then
    mkdir -p "$(dirname "$FASTRTPS_DEFAULT_PROFILES_FILE")"
    cat > "$FASTRTPS_DEFAULT_PROFILES_FILE" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="default" is_default_profile="true">
        <rtps>
            <useBuiltinTransports>true</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
XML
    echo "Fast DDS profile written: $FASTRTPS_DEFAULT_PROFILES_FILE"
fi

echo "Building ROS workspace..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

echo "Sourcing ROS workspace..."
source install/setup.bash
