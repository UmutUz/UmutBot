FROM ros:noetic-ros-base

# Install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.8 \
    ros-noetic-desktop \
    ros-noetic-navigation ros-noetic-nav2d \
    ros-noetic-slam-toolbox ros-noetic-robot-localization \
    ros-noetic-gmapping \
    ros-noetic-xacro \
    ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf ros-noetic-tf2 python3-rosdep ros-noetic-diff-drive-controller \
    nano joystick \
    python3-pip \
    ros-noetic-depthai-ros \
    ros-noetic-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Ensure the local directory is correctly referenced
COPY lg /lg

# Set the working directory
WORKDIR /lg

# Verify if the files were copied correctly
RUN ls -l /lg

# Run make and install
RUN make
RUN make install

# Install required Python package
RUN pip install --upgrade pip
RUN pip install --upgrade numpy
RUN pip install swig
RUN pip install rpi-lgpio
