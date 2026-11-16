#!/bin/bash
# Checks that everything needed for the aufdeckung simulation is available.
# Run it on the host first (Docker checks), then inside the dev container (simulation checks).
# Paste the full output when asking for help.

ok()   { printf "  [ OK ] %s\n" "$1"; }
bad()  { printf "  [FAIL] %s\n" "$1"; FAILED=1; }
info() { printf "  [info] %s\n" "$1"; }
FAILED=0

echo "== System"
info "$(grep PRETTY_NAME /etc/os-release 2>/dev/null | cut -d= -f2) / $(uname -m)"
info "repo branch: $(git rev-parse --abbrev-ref HEAD 2>/dev/null) @ $(git rev-parse --short HEAD 2>/dev/null)"

if [ -f /.dockerenv ] || grep -q docker /proc/1/cgroup 2>/dev/null; then
    echo "== Inside a container: checking the simulation stack"
    command -v ros2 >/dev/null && ok "ROS 2: $(printenv ROS_DISTRO || echo 'installed (not sourced?)')" || bad "ros2 not found (source /opt/ros/humble/setup.bash)"
    command -v gz >/dev/null && ok "Gazebo: $(gz sim --version 2>/dev/null | head -1)" || bad "gz (Gazebo) not found"
    PX4=${PX4_SOURCE_DIR:-/sim/PX4-Autopilot}
    [ -d "$PX4" ] && ok "PX4 source: $PX4 ($(git -C "$PX4" describe --tags 2>/dev/null))" || bad "PX4 not found at $PX4"
    [ -d "$PX4/build/px4_sitl_default" ] && ok "PX4 SITL already built" || info "PX4 SITL not built yet (first launch builds it, ~10 min)"
    python3 -c "import mavsdk" 2>/dev/null && ok "MAVSDK python" || bad "MAVSDK python missing (pip install mavsdk)"
    python3 -c "import aufdeckung.config" 2>/dev/null && ok "aufdeckung package installed" || bad "aufdeckung not installed (run ./scripts/setup-sim.sh)"
    [ -f install/setup.bash ] && ok "ROS workspace built (install/)" || bad "ROS workspace not built (run ./scripts/setup-sim.sh)"
    command -v tmuxinator >/dev/null && ok "tmuxinator" || bad "tmuxinator missing (apt install tmuxinator)"
    [ -n "$DISPLAY" ] && ok "DISPLAY=$DISPLAY (GUI windows possible)" || bad "DISPLAY not set: Gazebo GUI cannot open"
    if [ -e /dev/dxg ]; then
        ok "/dev/dxg present (WSL2 GPU rendering via DirectX 12)"
    elif [ -e /dev/dri ]; then
        ok "/dev/dri present (GPU rendering)"
    else
        info "no /dev/dri or /dev/dxg: Gazebo will render on CPU (slow)"
    fi
    if command -v glxinfo >/dev/null; then
        info "OpenGL $(glxinfo -B 2>/dev/null | grep -i 'renderer string' | sed 's/.*: //')"
    fi
else
    echo "== On the host: checking Docker"
    if command -v docker >/dev/null; then
        ok "docker: $(docker --version)"
        docker info >/dev/null 2>&1 && ok "docker daemon reachable without sudo" \
            || bad "cannot talk to docker: add yourself to the docker group, then log out/in"
        IMG=ghcr.io/aachen-drone-development-initiative/imav/dev:latest
        docker image inspect "$IMG" >/dev/null 2>&1 && ok "dev image present: $IMG" \
            || bad "dev image missing: docker login ghcr.io, then docker pull $IMG"
    else
        bad "docker not installed (sudo apt install -y docker.io)"
    fi
    command -v code >/dev/null && ok "VS Code installed" || info "VS Code not found (needed for 'Reopen in Container')"
    command -v nvidia-smi >/dev/null && info "NVIDIA GPU found: needs nvidia-container-toolkit for GPU in Docker"
    python3 -c "import aufdeckung.config" 2>/dev/null && ok "aufdeckung installed on host (headless sim works here too)" \
        || info "aufdeckung not installed on host (optional: cd aufdeckung && pip install -e '.[viz]')"
fi

echo
[ $FAILED -eq 0 ] && echo "All checks passed." || echo "Some checks failed - see [FAIL] lines above."
exit $FAILED
