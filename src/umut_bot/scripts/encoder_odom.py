#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion
import math
import time

# Robot Parameters (defaults; overridden from the ROS param server in odom_publisher)
WHEEL_RADIUS = 0.0335  # 3.35 cm
WHEEL_BASE = 0.255  # Distance between wheels (m)
ENCODER_TICKS_PER_REV = 403  # Encoder resolution

# Latest cumulative tick count received from the encoder feedback topics
encoder_latest_1 = 0
encoder_latest_2 = 0

# Tick count processed on the previous odometry update
encoder_prev_1 = 0
encoder_prev_2 = 0

prev_V = 0
prev_omega = 0
x = 0.0
y = 0.0
theta = 0.0
last_time = time.time()

def encoder_callback_1(msg):
    global encoder_latest_1
    encoder_latest_1 = msg.data  # Store latest encoder count

def encoder_callback_2(msg):
    global encoder_latest_2
    encoder_latest_2 = msg.data  # Store latest encoder count

def calculate_odometry():
    global x, y, theta, encoder_prev_1, encoder_prev_2, last_time, prev_V, prev_omega

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # Skip this update if no time elapsed (avoids divide-by-zero)
    if dt <= 0.0:
        return x, y, theta, prev_V, prev_omega

    # Read the latest encoder values delivered by the callbacks
    encoder_current_1 = encoder_latest_1
    encoder_current_2 = encoder_latest_2

    # Compute encoder differences (Δticks) since the previous update
    delta_ticks_1 = encoder_current_1 - encoder_prev_1
    delta_ticks_2 = encoder_current_2 - encoder_prev_2

    # Store current values for next iteration
    encoder_prev_1 = encoder_current_1
    encoder_prev_2 = encoder_current_2

    # Convert encoder ticks to distance
    distance_per_tick = (2 * math.pi * WHEEL_RADIUS) / ENCODER_TICKS_PER_REV
    delta_s1 = delta_ticks_1 * distance_per_tick
    delta_s2 = delta_ticks_2 * distance_per_tick

    # Compute linear and angular velocity
    V_L = delta_s1 / dt
    V_R = delta_s2 / dt
    V = (V_L + V_R) / 2
    omega = (V_R - V_L) / WHEEL_BASE

    # Apply smoothing filters
    ALPHA = 0.8
    V = ALPHA * V + (1 - ALPHA) * prev_V
    prev_V = V

    omega = ALPHA * omega + (1 - ALPHA) * prev_omega
    prev_omega = omega

    # Update robot pose
    delta_x = V * math.cos(theta) * dt
    delta_y = V * math.sin(theta) * dt
    delta_theta = omega * dt

    x += delta_x
    y += delta_y
    theta += delta_theta

    # Normalize theta
    theta = math.atan2(math.sin(theta), math.cos(theta))

    return x, y, theta, V, omega

def odom_publisher():
    global WHEEL_RADIUS, WHEEL_BASE, ENCODER_TICKS_PER_REV

    rospy.init_node('odom_publisher', anonymous=True)

    # Allow the launch file to override the robot parameters
    WHEEL_RADIUS = rospy.get_param('~wheel_radius', WHEEL_RADIUS)
    WHEEL_BASE = rospy.get_param('~wheel_base', WHEEL_BASE)
    ENCODER_TICKS_PER_REV = rospy.get_param('~ticks_per_revolution', ENCODER_TICKS_PER_REV)

    # Subscribe to encoder feedback
    rospy.Subscriber('/encoder_feedback_1', Int32, encoder_callback_1)
    rospy.Subscriber('/encoder_feedback_2', Int32, encoder_callback_2)

    # Publisher for odometry
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    
    # Transform broadcaster
    odom_broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        x, y, theta, V, omega = calculate_odometry()

        # Publish Transform for TF
        odom_broadcaster.sendTransform(
            (x, y, 0),
            tf.transformations.quaternion_from_euler(0, 0, theta),
            rospy.Time.now(),
            "base_link",
            "odom"
        )

        # Publish Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        odom_msg.pose.pose.orientation = Quaternion(*quaternion)
        odom_msg.twist.twist.linear.x = V
        odom_msg.twist.twist.angular.z = omega

        odom_pub.publish(odom_msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass
