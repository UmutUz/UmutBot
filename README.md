# UmutBot

Self-made differential drive robot project to test out my Ender 3 V2 printing capabilities and Raspberry Pi 5 ROS/visualisation limits. Scrambled together by practically non usable stuff only a student can afford! 

## Overview

UmutBot is a ROS Noetic differential drive robot. A Raspberry Pi 5 runs the ROS stack (inside Docker, since the Pi 5 doesn't natively support Noetic), while an Arduino Nano handles low-level motor control and encoder feedback over USB serial (the code refers to it as "pico" for historical reasons).

- **SLAM**: gmapping with an RPLIDAR
- **Navigation**: move_base with costmap + base local planner configs
- **Odometry**: wheel encoder odometry, updated on every Nano report (rf2o laser odometry available as an alternative)
- **Motor control**: `/cmd_vel` commands forwarded to the Arduino Nano over USB serial (`/dev/umutbot_nano`, see [USB device names](#usb-device-names))

## Hardware

- Raspberry Pi 5 (main computer)
- Arduino Nano (motor controller + encoder counting)
- L298N dual H-bridge motor driver
- RPLIDAR (on `/dev/rplidar`)
- Differential drive base with wheel encoders (3D printed on an Ender 3 V2)

### Arduino Nano pinout

| Component | Pin | Nano pin | Function |
|---|---|---|---|
| L298N | ENA | D9 (PWM) | Speed motor A (left) |
| | IN1 | D8 | Direction 1 motor A |
| | IN2 | D7 | Direction 2 motor A |
| | ENB | D5 (PWM) | Speed motor B (right) |
| | IN3 | D4 | Direction 1 motor B |
| | IN4 | D6 | Direction 2 motor B |
| Encoder 1 (motor A) | Phase A (yellow) | D2 (INT0) | Pulse counting |
| | Phase B (green) | D10 | Direction |
| Encoder 2 (motor B) | Phase A (yellow) | D3 (INT1) | Pulse counting |
| | Phase B (green) | D11 | Direction |
| Both encoders | VCC (blue) | 5V | Encoder power |
| | GND (black) | GND | Encoder ground |

The Nano talks to the Pi over USB at 115200 baud (`cmd_vel:<linear>:<angular>` in, `encoder1:<ticks>,encoder2:<ticks>` out at 20 Hz). Don't wire the Nano's TX straight to the Pi's GPIO UART: the Nano runs at 5 V and the Pi's pins only take 3.3 V.

Each wheel runs its own speed controller on the Nano (feedforward + PI on the encoder speed), so both wheels hold the commanded speed even when the supply sags. Wheel speeds are capped at `MAX_WHEEL_SPEED` (currently 0.15 m/s, measured on the bench). For bench tests the sketch also accepts `pwm:<left>:<right>` (raw PWM, controller bypassed) and `debug:1` (prints target/measured speed and PWM per wheel).

## Repository structure

```
firmware/
└── umutbot_nano/             # Arduino Nano sketch (motor driver + encoders)
src/
├── dockerfiles/
│   ├── Dockerfile           # ROS Noetic image with navigation/SLAM packages + lgpio
│   └── docker_container.sh  # Container run script
└── umut_bot/                # Main ROS package
    ├── config/              # move_base, costmap and local planner parameters
    ├── launch/
    │   ├── navigation.launch  # Full stack: lidar, gmapping, move_base, encoder odom, Nano node
    │   ├── pico.launch        # Nano motor controller node only
    │   └── rf2o.launch        # rf2o laser odometry (alternative to encoder odom)
    ├── scripts/
    │   ├── pico_ros.py        # Bridges /cmd_vel to the Nano over serial, publishes encoder feedback
    │   ├── encoder_odom.py    # Computes and publishes wheel odometry from encoder ticks
    │   ├── motor_driver.py    # Direct GPIO motor driver (legacy)
    │   └── odom_base.py       # odom → base_link transform helper
    ├── udev/                  # udev rule for stable /dev/umutbot_nano and /dev/rplidar names
    └── urdf/                  # Robot description (URDF + xacro)
```

## Running

### USB device names

The Nano and the RPLIDAR are both USB serial devices, so `ttyUSB0`/`ttyUSB1` can swap between boots. Install the udev rule on the Pi host once to get stable names:

```bash
sudo cp src/umut_bot/udev/99-umutbot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/umutbot_nano /dev/rplidar
```

If a link is missing, check the vendor/product ID with `lsusb` and fix it in the rule. Without the rule, pass the ports by hand: `roslaunch umut_bot navigation.launch nano_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0`.

The container needs access to both devices, e.g. `--device=/dev/umutbot_nano --device=/dev/rplidar` (or `-v /dev:/dev --privileged`, which also survives re-plugging).

### Docker + ROS

Build the image (ROS Noetic on the Pi 5), then start a container with the USB devices and the workspace mounted:

```bash
cd src/dockerfiles
./docker_container.sh    # builds the image "noetic"
docker run -it --net=host --privileged -v /dev:/dev -v ~/UmutBot:/UmutBot noetic bash
```

Noetic is end-of-life and the old packages.ros.org signing key has expired, so the Dockerfile switches apt to the final Noetic snapshot repository. The image has no Pi GPIO libraries, since the Nano drives the motors over USB.

Inside the container, build the workspace and launch the full stack:

```bash
catkin_make
source devel/setup.bash
roslaunch umut_bot navigation.launch
```

This starts the RPLIDAR node, gmapping SLAM, move_base, encoder odometry, and the Nano motor bridge. Send goals via move_base (e.g. from RViz) or drive manually by publishing to `/cmd_vel` (e.g. `teleop_twist_keyboard`).

## License

MIT — see [LICENSE](LICENSE).
