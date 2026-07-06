# UmutBot

Self-made differential drive robot project to test out my Ender 3 V2 printing capabilities and Raspberry Pi 5 ROS/visualisation limits. Scrambled together by practically non usable stuff only a student can afford! 

## Overview

UmutBot is a ROS Noetic differential drive robot. A Raspberry Pi 5 runs the ROS stack (inside Docker, since the Pi 5 doesn't natively support Noetic), while an ESP32 handles low-level motor control and encoder feedback over UART (the code refers to it as "pico" for historical reasons).

- **SLAM**: gmapping with an RPLIDAR
- **Navigation**: move_base with costmap + base local planner configs
- **Odometry**: wheel encoder odometry (rf2o laser odometry available as an alternative)
- **Motor control**: `/cmd_vel` commands forwarded to the ESP32 over serial (`/dev/ttyAMA0`)

## Hardware

- Raspberry Pi 5 (main computer)
- ESP32 (motor controller + encoder counting)
- RPLIDAR (on `/dev/ttyUSB0`)
- Differential drive base with wheel encoders (3D printed on an Ender 3 V2)

## Repository structure

```
src/
├── dockerfiles/
│   ├── Dockerfile           # ROS Noetic image with navigation/SLAM packages + lgpio
│   └── docker_container.sh  # Container run script
└── umut_bot/                # Main ROS package
    ├── config/              # move_base, costmap and local planner parameters
    ├── launch/
    │   ├── navigation.launch  # Full stack: lidar, gmapping, move_base, encoder odom, ESP32 node
    │   ├── move_base.launch   # Navigation stack only
    │   ├── pico.launch        # ESP32 motor controller node only
    │   └── rf2o.launch        # rf2o laser odometry (alternative to encoder odom)
    ├── scripts/
    │   ├── pico_ros.py        # Bridges /cmd_vel to the ESP32 over UART, publishes encoder feedback
    │   ├── encoder_odom.py    # Computes and publishes wheel odometry from encoder ticks
    │   ├── motor_driver.py    # Direct GPIO motor driver (legacy)
    │   └── odom_base.py       # odom → base_link transform helper
    └── urdf/                  # Robot description (URDF + xacro)
```

## Running

Build and start the Docker container (ROS Noetic on the Pi 5):

```bash
cd src/dockerfiles
docker build -t umutbot .
./docker_container.sh
```

Inside the container, build the workspace and launch the full stack:

```bash
catkin_make
source devel/setup.bash
roslaunch umut_bot navigation.launch
```

This starts the RPLIDAR node, gmapping SLAM, move_base, encoder odometry, and the ESP32 motor bridge. Send goals via move_base (e.g. from RViz) or drive manually by publishing to `/cmd_vel` (e.g. `teleop_twist_keyboard`).

## License

MIT — see [LICENSE](LICENSE).
