FROM ros:noetic-ros-base-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop\
    ros-noetic-navigation2 ros-noetic-nav2-bringup \
    ros-noetic-slam-toolbox ros-noetic-robot-localization \
    ros-noetic-xacro \
    ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf-transformations python3-rosdep2 \
    nano joystick bc \
    python3-pip \
    imagemagick \
    alsa-base \
    alsa-utils \
    libsndfile1-dev \
    libportaudio2 \
    ros-noetic-depthai-ros \
    ros-noetic-joint-state-publisher \
    ros-noetic-rtabmap ros-noetic-rtabmap-conversions ros-noetic-rtabmap-examples \
    ros-noetic-rtabmap-launch ros-noetic-rtabmap-msgs ros-noetic-rtabmap-odom \
    ros-noetic-rtabmap-python ros-noetic-rtabmap-slam ros-noetic-rtabmap-sync ros-noetic-rtabmap-util \
    iputils-ping wget psmisc \
    && rm -rf /var/lib/apt/lists/*