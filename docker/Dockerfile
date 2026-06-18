# ============================================================
# 1. BASE IMAGE MAPS
# ============================================================
FROM osrf/ros:jazzy-desktop AS base-amd64
FROM ros:jazzy-ros-base AS base-arm64

# ============================================================
# 2. ARCHITECTURE ROUTING
# ============================================================
ARG TARGETARCH
FROM base-${TARGETARCH} AS finalized_root
ENV DEBIAN_FRONTEND=noninteractive

# ============================================================
# 3. CORE DEPENDENCIES & DRIVERS (Shared)
# ============================================================
RUN apt-get update && apt-get install -y \
    git cmake build-essential \
    libfmt-dev \
    python3-rosdep python3-colcon-common-extensions \
    python3-pip python3-vcstool \
    nano vim wget curl unzip \
    x11-apps mesa-utils \
    libusb-1.0-0-dev libserial-dev \
    libglib2.0-dev libxerces-c-dev libtinyxml2-dev \
    python3-opencv \
    joystick evtest input-utils \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# 4. PACKAGE-DECLARED ROS DEPENDENCIES (driven by package.xml)
# ============================================================
# gz_ros2_control / ros_gz_bridge / ros_gz_sim are skipped here because they're
# PC-only (installed conditionally in section 5, not wanted on the Pi image).
# explore_lite is skipped because it has no rosdep key for jazzy yet
# (m-explore-ros2 isn't packaged) - build it from source if/when it's needed.
RUN rosdep init || true && rosdep update
COPY src/ /tmp/ros2_ws/src/
RUN apt-get update && rosdep install --from-paths /tmp/ros2_ws/src --ignore-src -r -y \
    --skip-keys "gz_ros2_control ros_gz_bridge ros_gz_sim explore_lite" \
    && rm -rf /var/lib/apt/lists/* /tmp/ros2_ws
# cv_bridge isn't declared in any package.xml yet (reserved for the upcoming yolo node)
RUN apt-get update && apt-get install -y ros-jazzy-cv-bridge && rm -rf /var/lib/apt/lists/*

# ============================================================
# 5. OPTIONAL GAZEBO SIMULATION (PC ONLY)
# ============================================================
# CRITICAL FIX: Re-declare ARG within this execution block layer
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        apt-get update && apt-get install -y \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-gz-ros2-control \
        && rm -rf /var/lib/apt/lists/*; \
    fi

# ============================================================
# 6. GRAPHICS HARDWARE EXTRACTION
# ============================================================
# CRITICAL FIX: Re-declare ARG here so the condition isn't skipped
ARG TARGETARCH
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        apt-get update && apt-get install -y libglvnd-dev && rm -rf /var/lib/apt/lists/*; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
        apt-get update && apt-get install -y libraspberrypi-bin || true && rm -rf /var/lib/apt/lists/*; \
    fi

# ============================================================
# 7. YDLIDAR SDK SOURCE COMPILATION
# ============================================================
# Note: Fixed broken source URI from your template code block
RUN git clone https://github.com/ydlidar/YDLidar-SDK.git /opt/YDLidar-SDK
RUN cd /opt/YDLidar-SDK && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig
ENV ydlidar_sdk_DIR=/opt/YDLidar-SDK

# ============================================================
# 8. WORKSPACE HOUSEKEEPING
# ============================================================
RUN touch /usr/lib/python3/dist-packages/numpy/_typing/.colconignore || true

# ============================================================
# 9. RUNTIME PIPELINE ANCHORS
# ============================================================
WORKDIR /ros2_ws
ENV TURTLEBOT3_MODEL=burger

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
RUN echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> ~/.bashrc

CMD ["/bin/bash"]
