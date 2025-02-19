#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import serial

# Initialize UART connection
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

# Global variables to store encoder readings
encoder_value_1 = 0
encoder_value_2 = 0

def cmd_vel_callback(msg):
    """Send velocity command to Pico."""
    linear = msg.linear.x
    angular = msg.angular.z
    cmd = f"cmd_vel:{linear}:{angular}\n"
    ser.write(cmd.encode('utf-8'))

def read_encoders():
    """Read encoder feedback from Pico."""
    global encoder_value_1, encoder_value_2
    line = ser.readline().decode('utf-8').strip()
    if line.startswith("encoder1:") and "encoder2:" in line:
        try:
            parts = line.split(",")
            encoder_value_1 = int(parts[0].split(":")[1])
            encoder_value_2 = int(parts[1].split(":")[1])
        except ValueError:
            pass

def main():
    rospy.init_node('pico_motor_controller')

    # Subscribers
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    # Publishers for both encoders
    encoder_pub_1 = rospy.Publisher('/encoder_feedback_1', Int32, queue_size=10)
    encoder_pub_2 = rospy.Publisher('/encoder_feedback_2', Int32, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        read_encoders()
        encoder_pub_1.publish(encoder_value_1)
        encoder_pub_2.publish(encoder_value_2)
        rate.sleep()

    ser.close()

if __name__ == "__main__":
    main()
