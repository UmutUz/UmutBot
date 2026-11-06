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

# lgpio C library, needed by rpi-lgpio below (RPi.GPIO on the Pi 5; only the
# old motor_driver.py uses it). Cloned at a fixed tag so the build works from
# a plain git checkout instead of needing a local lg/ folder.
RUN git clone --depth 1 --branch v0.2.2 https://github.com/joan2937/lg /lg
WORKDIR /lg
RUN make && make install

# Install required Python package
RUN pip install --upgrade pip
RUN pip install --upgrade numpy
RUN pip install swig
RUN pip install rpi-lgpio
