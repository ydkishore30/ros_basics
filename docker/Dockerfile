FROM osrf/ros:jazzy-desktop

# ----------------------------
# Basic tools + dependencies
# ----------------------------
RUN apt update && apt install -y \
    git \
    cmake \
    build-essential \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-pip \
    nano \
    vim \
    x11-apps \
    usbutils \
    udev \
    libusb-1.0-0-dev \
    libserial-dev \
    libglib2.0-dev \
    libxerces-c-dev \
    libtinyxml2-dev \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------
# ROS Gazebo + control packages
# ----------------------------
RUN apt update && apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-diff-drive-controller \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------
# rosdep
# ----------------------------
RUN rosdep init || true
RUN rosdep update

# ============================================================
# YDLIDAR SDK (IMPORTANT FIX HERE)
# ============================================================

RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git /opt/YDLidar-SDK

RUN cd /opt/YDLidar-SDK && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && ldconfig

# >>> IMPORTANT: make ROS aware of SDK <<<
ENV CMAKE_PREFIX_PATH=/opt/YDLidar-SDK:$CMAKE_PREFIX_PATH
ENV ydlidar_sdk_DIR=/opt/YDLidar-SDK
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# ----------------------------
# ROS2 workspace
# ----------------------------
WORKDIR /ros2_ws
COPY src /ros2_ws/src

RUN rm -rf build install log || true

# ----------------------------
# Install dependencies
# ----------------------------
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy"

# ----------------------------
# Build workspace (IMPORTANT FIX HERE)
# ----------------------------
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --cmake-args -Dydlidar_sdk_DIR=/opt/YDLidar-SDK"

# ----------------------------
# Environment variables
# ----------------------------
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
ENV AMENT_PREFIX_PATH=/opt/ros/jazzy:$AMENT_PREFIX_PATH
ENV ROS_PACKAGE_PATH=/opt/ros/jazzy/share:$ROS_PACKAGE_PATH

# ----------------------------
# Auto source
# ----------------------------
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]