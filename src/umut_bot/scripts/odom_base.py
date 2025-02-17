#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry

class OdomToBaseLinkTF:
    def __init__(self):
        rospy.init_node('odom_to_base_link_tf_publisher')

        # Initialize the Transform Broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Parameters for the robot's pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Subscribe to the odometry topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo("odom_to_base_link_tf_publisher started")
        rospy.spin()

    def odom_callback(self, msg):
        # Extract position and orientation from the Odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        self.theta = tf.transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])[2]  # Extract yaw

        # Broadcast the transform from odom -> base_link
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0),  # Translation (x, y, z)
            (orientation.x, orientation.y, orientation.z, orientation.w),  # Quaternion
            rospy.Time.now(),
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    try:
        OdomToBaseLinkTF()
    except rospy.ROSInterruptException:
        pass
