#!/bin/bash
# Launch Gazebo simulation inside the PC Docker container.
# Run this from your terminal (not from Claude/VSCode terminal) so the GUI appears.
#
# Usage:  ./sim.sh          — build workspace if needed, then launch
#         ./sim.sh --build  — force a full rebuild before launching

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Allow the Docker container (runs as root) to draw on your X display
echo "Granting X11 access to Docker..."
xhost +local:docker

# Use whatever display is active in this terminal
CURRENT_DISPLAY="${DISPLAY:-:1}"

FORCE_BUILD=0
if [[ "$1" == "--build" ]]; then
    FORCE_BUILD=1
fi

BUILD_CMD=""
if [[ "$FORCE_BUILD" == "1" ]]; then
    BUILD_CMD="echo 'Building workspace...' && \
    colcon build \
      --packages-skip micro_ros_setup micro_ros_agent micro_ros_msgs my_robot_rl \
      --cmake-args -DCMAKE_BUILD_TYPE=Release && "
else
    # Build only if install hasn't been done inside the container yet
    BUILD_CMD='if [ ! -f /ros2_ws/install/my_robot_bringup/share/my_robot_bringup/local_setup.bash ] || \
                 [ "$(stat -c %i /ros2_ws/install/my_robot_bringup/share/my_robot_bringup/local_setup.bash)" = "0" ]; then
                    echo "Building workspace..."
                    colcon build \
                      --packages-skip micro_ros_setup micro_ros_agent micro_ros_msgs my_robot_rl \
                      --cmake-args -DCMAKE_BUILD_TYPE=Release
               fi && '
fi

echo "Starting simulation container (display: $CURRENT_DISPLAY)..."
docker compose --profile pc run --rm \
    -e DISPLAY="$CURRENT_DISPLAY" \
    ros-pc bash -c "
        set -e
        source /opt/ros/jazzy/setup.bash
        cd /ros2_ws
        ${BUILD_CMD}
        source install/setup.bash
        echo 'Launching Gazebo simulation...'
        ros2 launch my_robot_bringup my_robot_gazebo.launch.py
    "
