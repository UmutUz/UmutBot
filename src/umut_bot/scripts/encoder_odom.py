#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import RPi.GPIO as GPIO
import math
import time


class EncoderOdometryNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('encoder_odometry_node', anonymous=True)

        # Parameters
        self.encoder_left_pin = rospy.get_param('~encoder_left_pin', 16)  # GPIO pin for left encoder
        self.encoder_right_pin = rospy.get_param('~encoder_right_pin', 26)  # GPIO pin for right encoder
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0315)  # Wheel radius in meters
        self.wheel_base = rospy.get_param('~wheel_base', 0.155)  # Distance between wheels in meters
        self.ticks_per_revolution = rospy.get_param('~ticks_per_revolution', 20)  # Encoder ticks per revolution

        # Set up GPIO
        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setup(self.encoder_left_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.encoder_right_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Encoder tick counters
        self.left_ticks = 0
        self.right_ticks = 0

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = rospy.Time.now()

        # ROS Publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

        # Set up GPIO interrupts
        GPIO.add_event_detect(self.encoder_left_pin, GPIO.RISING, callback=self.left_tick_callback)
        GPIO.add_event_detect(self.encoder_right_pin, GPIO.RISING, callback=self.right_tick_callback)

        # Timer to update odometry
        self.odom_timer = rospy.Timer(rospy.Duration(0.2), self.update_odometry)

        rospy.loginfo("Encoder Odometry Node Initialized")

    def left_tick_callback(self, channel):
        """Callback for left encoder tick."""
        self.left_ticks += 1

    def right_tick_callback(self, channel):
        """Callback for right encoder tick."""
        self.right_ticks += 1

    def update_odometry(self, event):
        """Calculate and publish odometry."""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Calculate distance per tick
        distance_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_revolution

        # Calculate wheel distances
        left_distance = self.left_ticks * distance_per_tick
        right_distance = self.right_ticks * distance_per_tick

        # Reset tick counters
        self.left_ticks = 0
        self.right_ticks = 0

        # Compute robot motion
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        # Update pose
        self.theta += delta_theta
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.theta / 2.0),
            w=math.cos(self.theta / 2.0)
        )

        # Set velocity
        odom_msg.twist.twist.linear.x = distance / dt
        odom_msg.twist.twist.angular.z = delta_theta / dt

        self.odom_pub.publish(odom_msg)

        #rospy.loginfo(f"Odometry: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")

    def cleanup(self):
        """Clean up GPIO when shutting down."""
        GPIO.cleanup()


def main():
    try:
        node = EncoderOdometryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()


if __name__ == "__main__":
    main()
