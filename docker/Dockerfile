FROM osrf/ros:jazzy-desktop

# ============================================================
# Core dependencies
# ============================================================
RUN apt update && apt install -y \
    git cmake build-essential \
    python3-rosdep python3-colcon-common-extensions \
    python3-pip python3-vcstool \
    nano vim wget curl unzip \
    x11-apps mesa-utils \
    libusb-1.0-0-dev libserial-dev \
    libglib2.0-dev libxerces-c-dev libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Gazebo + ROS2 control + Nav2 + SLAM
# ============================================================
RUN apt update && apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-xacro \
    ros-jazzy-rviz2 \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-turtlebot3 \
    ros-jazzy-turtlebot3-msgs \
    ros-jazzy-turtlebot3-gazebo \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# rosdep
# ============================================================
RUN rosdep init || true
RUN rosdep update

# ============================================================
# YDLidar SDK
# ============================================================
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git /opt/YDLidar-SDK

RUN cd /opt/YDLidar-SDK && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

ENV ydlidar_sdk_DIR=/opt/YDLidar-SDK

# ============================================================
# Simulation assets
# ============================================================
RUN mkdir -p /simulation_assets/worlds \
    /simulation_assets/models \
    /simulation_assets/maps

# ============================================================
# ROS2 workspace
# ============================================================
WORKDIR /ros2_ws
COPY src /ros2_ws/src

RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy'

RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install'

# ============================================================
# Gazebo paths
# ============================================================
ENV TURTLEBOT3_MODEL=burger


# ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib

# ============================================================
# ROS sourcing
# ============================================================
RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
RUN echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc

CMD ["/bin/bash"]