#!/usr/bin/env python3
"""Wheel odometry from the cumulative encoder counts the Nano reports.

pico_ros.py publishes /encoder_feedback_1 (left) and /encoder_feedback_2
(right) for every line the Nano sends (20 Hz). Odometry is updated once per
such report, with dt taken from the arrival times of the data. That keeps
the reported velocity smooth instead of jumping between "no new ticks" and
"a whole report at once", which happened when this ran at a fixed 100 Hz.

Publishes /odom and the odom -> base_link transform (20 Hz).
"""

import math
import threading

import rospy
import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32

# A jump bigger than this between two reports can't be real wheel motion
# (~16 ticks per report at full speed): the Nano restarted and its counters
# went back to 0. The pose is kept and counting continues from the new values.
MAX_TICKS_PER_REPORT = 500

# Velocity is reported as 0 if no encoder data arrives for this long
DATA_TIMEOUT = 0.25  # s

# Weight of the newest sample in the velocity low-pass filter (twist only;
# the pose is integrated from the raw tick deltas)
ALPHA = 0.8


class EncoderOdometry:
    def __init__(self):
        wheel_radius = rospy.get_param('~wheel_radius', 0.035)
        self.wheel_base = rospy.get_param('~wheel_base', 0.255)
        ticks_per_rev = rospy.get_param('~ticks_per_revolution', 403)
        self.m_per_tick = 2 * math.pi * wheel_radius / ticks_per_rev

        # Written by the subscriber threads, read by the main loop
        self.lock = threading.Lock()
        self.latest = [None, None]      # newest tick count per wheel
        self.fresh = [False, False]     # updated since the last odometry step?
        self.stamp = None               # arrival time of the newest data

        self.prev = None                # tick counts used in the last step
        self.prev_stamp = None
        self.x = self.y = self.theta = 0.0
        self.v = self.omega = 0.0

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/encoder_feedback_1', Int32, self.encoder_cb, 0)
        rospy.Subscriber('/encoder_feedback_2', Int32, self.encoder_cb, 1)

    def encoder_cb(self, msg, wheel):
        with self.lock:
            self.latest[wheel] = msg.data
            self.fresh[wheel] = True
            self.stamp = rospy.Time.now()

    def step(self):
        """Run one odometry update once both wheels have new data.

        Returns True if the pose was updated and should be published.
        """
        with self.lock:
            if not all(self.fresh):
                return False
            ticks = list(self.latest)
            stamp = self.stamp
            self.fresh = [False, False]

        if self.prev is None:           # first report: only remember it
            self.prev, self.prev_stamp = ticks, stamp
            return False

        d_left = ticks[0] - self.prev[0]
        d_right = ticks[1] - self.prev[1]
        dt = (stamp - self.prev_stamp).to_sec()
        self.prev, self.prev_stamp = ticks, stamp

        if abs(d_left) > MAX_TICKS_PER_REPORT or abs(d_right) > MAX_TICKS_PER_REPORT:
            rospy.logwarn("Encoder counts jumped by (%d, %d) - Nano restarted? Resyncing.",
                          d_left, d_right)
            return False

        ds_left = d_left * self.m_per_tick
        ds_right = d_right * self.m_per_tick
        ds = (ds_left + ds_right) / 2.0
        dtheta = (ds_right - ds_left) / self.wheel_base

        # Integrate the pose from the tick deltas, using the heading halfway
        # through the step (exact for constant-curvature arcs to first order)
        heading = self.theta + dtheta / 2.0
        self.x += ds * math.cos(heading)
        self.y += ds * math.sin(heading)
        self.theta = math.atan2(math.sin(self.theta + dtheta), math.cos(self.theta + dtheta))

        if dt > 0.0:
            self.v = ALPHA * (ds / dt) + (1 - ALPHA) * self.v
            self.omega = ALPHA * (dtheta / dt) + (1 - ALPHA) * self.omega
        return True

    def check_timeout(self, now):
        """Report zero velocity if the encoder data stopped arriving."""
        if self.prev_stamp is None:
            return
        if (now - self.prev_stamp).to_sec() > DATA_TIMEOUT and (self.v or self.omega):
            rospy.logwarn_throttle(5, "No encoder data for %.2f s", DATA_TIMEOUT)
            self.v = self.omega = 0.0

    def publish(self, stamp):
        q = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        self.tf_broadcaster.sendTransform((self.x, self.y, 0), q, stamp, "base_link", "odom")

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(*q)
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega
        self.odom_pub.publish(odom)

    def spin(self):
        rate = rospy.Rate(100)  # only polls for new encoder data
        while not rospy.is_shutdown():
            if self.step():
                self.publish(self.prev_stamp)
            else:
                self.check_timeout(rospy.Time.now())
            rate.sleep()


def main():
    rospy.init_node('odom_publisher', anonymous=True)
    EncoderOdometry().spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
