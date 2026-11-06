FROM ros:noetic-ros-base

# Noetic is end-of-life (May 2025), its packages now come from the final
# snapshot repository. Older ros:noetic images still point at packages.ros.org
# with a signing key that has expired (apt-get update fails with
# "EXPKEYSIG F42ED6FBAB17C654"), so switch to the snapshot repo and its key
# (valid until 2027-06-01), the same way the current official image does.
RUN set -eux; \
    rm -f /etc/apt/sources.list.d/ros*.list; \
    key='4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA'; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
    mkdir -p /usr/share/keyrings; \
    gpg --batch --export "$key" > /usr/share/keyrings/ros1-snapshots-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME"; \
    echo "deb [ signed-by=/usr/share/keyrings/ros1-snapshots-archive-keyring.gpg ] http://snapshots.ros.org/noetic/final/ubuntu focal main" \
        > /etc/apt/sources.list.d/ros1-snapshots.list

# ROS Noetic packages and tools
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
    git \
    python3-pip \
    python3-serial \
    ros-noetic-depthai-ros \
    ros-noetic-joint-state-publisher \
    && rm -rf /var/lib/apt/lists/*

# Foxglove bridge, to view the robot from the PC in Foxglove
# (ws://raspberrypi:8765). Its own step, so adding it doesn't invalidate the
# big package layer above and the rebuild stays fast.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# No Pi GPIO libraries (lgpio / rpi-lgpio) anymore: the Arduino Nano drives
# the motors over USB. Only the old motor_driver.py used RPi.GPIO, and it is
# commented out in navigation.launch. lgpio also doesn't compile here, since
# Ubuntu focal's kernel headers lack the GPIO v2 API it needs.

# Install required Python package
RUN pip install --upgrade pip
RUN pip install --upgrade numpy
