FROM ros:noetic-ros-base

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop \
    ros-noetic-navigation ros-noetic-nav2d \
    ros-noetic-slam-toolbox ros-noetic-robot-localization \
    ros-noetic-xacro \
    ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf ros-noetic-tf2 python3-rosdep \
    nano joystick \
    python3-pip \
     ros-noetic-depthai-ros \
    ros-noetic-joint-state-publisher \
     && rm -rf /var/lib/apt/lists/*
