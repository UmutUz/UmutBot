#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gpiozero import Motor, PWMLED
from time import sleep

class L298NMotorDriver:
    def __init__(self):
        # GPIO pin setup using gpiozero Motor class
        self.motor1_in1 = rospy.get_param('~motor1_in1', 17)  # Default GPIO 17
        self.motor1_in2 = rospy.get_param('~motor1_in2', 27)  # Default GPIO 27
        self.motor1_en = rospy.get_param('~motor1_en', 22)    # Default GPIO 22

        self.motor2_in1 = rospy.get_param('~motor2_in1', 23)  # Default GPIO 23
        self.motor2_in2 = rospy.get_param('~motor2_in2', 24)  # Default GPIO 24
        self.motor2_en = rospy.get_param('~motor2_en', 25)    # Default GPIO 25

        # Setup motors using gpiozero Motor and PWMLED classes
        self.motor1 = Motor(self.motor1_in1, self.motor1_in2)
        self.motor2 = Motor(self.motor2_in1, self.motor2_in2)

        # PWM for motor speed control (gpiozero PWMLED)
        self.motor1_pwm = PWMLED(self.motor1_en)
        self.motor2_pwm = PWMLED(self.motor2_en)

        # ROS setup
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/stop_motor', Bool, self.stop_motor_callback)

        self.stop_motor = False

    def cmd_vel_callback(self, msg):
        if self.stop_motor:
            self.stop()
            return

        linear_x = msg.linear.x  # Forward/backward
        angular_z = msg.angular.z  # Left/right rotation

        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        self.set_motor_speeds(left_speed, right_speed)

    def stop_motor_callback(self, msg):
        self.stop_motor = msg.data
        if self.stop_motor:
            self.stop()

    def set_motor_speeds(self, left_speed, right_speed):
        self.set_motor_speed(self.motor1, self.motor1_pwm, left_speed)
        self.set_motor_speed(self.motor2, self.motor2_pwm, right_speed)

    def set_motor_speed(self, motor, pwm, speed):
        if speed > 0:
            motor.forward()
        elif speed < 0:
            motor.backward()
        else:
            motor.stop()

        # Adjust the PWM (duty cycle) for speed control
        pwm.value = min(abs(speed), 1.0)

    def stop(self):
        self.motor1.stop()
        self.motor2.stop()
        self.motor1_pwm.off()
        self.motor2_pwm.off()

    def cleanup(self):
        self.stop()

if __name__ == '__main__':
    rospy.init_node('l298n_motor_driver', anonymous=True)
    driver = L298NMotorDriver()

    rospy.on_shutdown(driver.cleanup)
    rospy.spin()
