#!/usr/bin/env python3
"""Bridge between ROS and the Arduino Nano motor controller.

Forwards /cmd_vel to the Nano over serial and publishes the cumulative
encoder counts it reports on /encoder_feedback_1 and /encoder_feedback_2.
(The "pico" name is historical; the firmware lives in firmware/umutbot_nano.)

Serial protocol (ASCII, one message per line):
  Pi   -> Nano : cmd_vel:<linear m/s>:<angular rad/s>
  Nano -> Pi   : encoder1:<ticks>,encoder2:<ticks>
"""

import time

import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


def parse_encoder_line(raw):
    """Return (ticks1, ticks2) from an encoder line, or None if it's not one."""
    line = raw.decode('ascii', errors='ignore').strip()
    if not line.startswith('encoder1:'):
        return None
    try:
        part1, part2 = line.split(',')
        if not part2.startswith('encoder2:'):
            return None
        return int(part1.split(':')[1]), int(part2.split(':')[1])
    except (ValueError, IndexError):
        return None


def main():
    rospy.init_node('pico_motor_controller')

    port = rospy.get_param('~port', '/dev/umutbot_nano')
    baud = rospy.get_param('~baud', 115200)

    ser = serial.Serial(port, baud, timeout=0.1)
    # Opening the port resets the Nano (DTR line); wait for the bootloader
    # to hand over to the sketch, then drop anything sent during boot.
    time.sleep(2.0)
    ser.reset_input_buffer()
    rospy.loginfo("Connected to Arduino Nano on %s @ %d baud", port, baud)

    def send_cmd(linear, angular):
        try:
            ser.write(f"cmd_vel:{linear:.3f}:{angular:.3f}\n".encode('ascii'))
        except serial.SerialException as e:
            rospy.logwarn_throttle(5, "Serial write failed: %s", e)

    rospy.Subscriber('/cmd_vel', Twist,
                     lambda msg: send_cmd(msg.linear.x, msg.angular.z))

    encoder_pub_1 = rospy.Publisher('/encoder_feedback_1', Int32, queue_size=10)
    encoder_pub_2 = rospy.Publisher('/encoder_feedback_2', Int32, queue_size=10)

    # Publish every line the Nano sends (20 Hz). Reading as fast as it
    # arrives keeps the serial buffer from backing up and adding lag.
    while not rospy.is_shutdown():
        ticks = parse_encoder_line(ser.readline())
        if ticks is None:
            continue
        encoder_pub_1.publish(ticks[0])
        encoder_pub_2.publish(ticks[1])

    send_cmd(0.0, 0.0)
    ser.close()


if __name__ == "__main__":
    main()
