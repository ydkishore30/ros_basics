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
# 4. NAVIGATION, ROS2 CONTROL & RUNTIME MANDATORY PACKAGES
# ============================================================
RUN apt-get update && apt-get install -y \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-cv-bridge \
    ros-jazzy-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

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
        ros-jazzy-turtlebot3 \
        ros-jazzy-turtlebot3-msgs \
        ros-jazzy-turtlebot3-gazebo \
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
# 8. DEPENDENCY REGISTRY CONFIGURATION
# ============================================================
RUN rosdep init || true && rosdep update
RUN touch /usr/lib/python3/dist-packages/numpy/_typing/.colconignore || true

# ============================================================
# 9. RUNTIME PIPELINE ANCHORS
# ============================================================
WORKDIR /ros2_ws
ENV TURTLEBOT3_MODEL=burger

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
RUN echo 'if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi' >> ~/.bashrc

CMD ["/bin/bash"]
