#!/bin/bash
# Launch real-hardware bringup (with YDLidar X2) inside the Pi Docker container.
#
# Usage:  ./robot.sh               — launch bringup with lidar
#         ./robot.sh --build       — rebuild workspace first, then launch
#         ./robot.sh --no-lidar    — launch without the lidar
#         ./robot.sh <arg:=value>  — extra launch args are passed through

set -e

CONTAINER=ros_basic_container
USE_LIDAR=true
FORCE_BUILD=0
EXTRA_ARGS=()

for arg in "$@"; do
    case "$arg" in
        --build)    FORCE_BUILD=1 ;;
        --no-lidar) USE_LIDAR=false ;;
        *)          EXTRA_ARGS+=("$arg") ;;
    esac
done

# Fixed USB-port paths (see my_robot_real.launch.py)
LIDAR_PORT=/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2:1.0-port0
ESP32_PORT=/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0

[ -e "$ESP32_PORT" ] || { echo "ERROR: ESP32 not found — must be in Pi USB port 1.3"; exit 1; }
if [[ "$USE_LIDAR" == "true" && ! -e "$LIDAR_PORT" ]]; then
    echo "ERROR: YDLidar not found — must be in Pi USB port 1.2 (or use --no-lidar)"
    exit 1
fi

# Start the container if it isn't running
if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" != "true" ]; then
    echo "Starting container $CONTAINER..."
    docker start "$CONTAINER"
fi

BUILD_CMD=""
if [[ "$FORCE_BUILD" == "1" ]]; then
    BUILD_CMD="echo 'Building workspace...' && \
    colcon build --symlink-install \
      --packages-skip micro_ros_setup micro_ros_agent micro_ros_msgs my_robot_rl && "
fi

echo "Launching real robot bringup (use_lidar:=$USE_LIDAR)..."
# -it so Ctrl+C is forwarded and all nodes shut down cleanly
exec docker exec -it "$CONTAINER" bash -c "
    set -e
    source /opt/ros/jazzy/setup.bash
    cd /ros2_ws
    ${BUILD_CMD}
    source install/setup.bash
    ros2 launch my_robot_bringup my_robot_real.launch.py use_lidar:=$USE_LIDAR ${EXTRA_ARGS[*]}
"
