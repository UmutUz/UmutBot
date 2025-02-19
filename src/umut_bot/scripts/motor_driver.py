#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

class L298NMotorDriver:
    def __init__(self):
        # GPIO pin setup
        self.motor1_in1 = rospy.get_param('~motor1_in1', 27)  # Default GPIO 17
        self.motor1_in2 = rospy.get_param('~motor1_in2', 17)  # Default GPIO 27
        self.motor1_en = rospy.get_param('~motor1_en', 12)    # Default GPIO 22

        self.motor2_in1 = rospy.get_param('~motor2_in1', 23)  # Default GPIO 23
        self.motor2_in2 = rospy.get_param('~motor2_in2', 24)  # Default GPIO 24
        self.motor2_en = rospy.get_param('~motor2_en', 13)    # Default GPIO 25

        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.motor1_in1, self.motor1_in2, self.motor1_en,
                    self.motor2_in1, self.motor2_in2, self.motor2_en], GPIO.OUT)

        self.motor1_pwm = GPIO.PWM(self.motor1_en, 1000)  # 100 Hz PWM frequency
        self.motor2_pwm = GPIO.PWM(self.motor2_en, 1000)

        self.motor1_pwm.start(0)
        self.motor2_pwm.start(0)

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
        self.set_motor_speed(self.motor1_in1, self.motor1_in2, self.motor1_pwm, left_speed)
        self.set_motor_speed(self.motor2_in1, self.motor2_in2, self.motor2_pwm, right_speed)

    def set_motor_speed(self, in1, in2, pwm, speed):
        if speed > 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)

        pwm.ChangeDutyCycle(min(abs(speed) * 100, 100))

    def stop(self):
        GPIO.output([self.motor1_in1, self.motor1_in2, self.motor2_in1, self.motor2_in2], GPIO.LOW)
        self.motor1_pwm.ChangeDutyCycle(0)
        self.motor2_pwm.ChangeDutyCycle(0)

    def cleanup(self):
        self.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    rospy.init_node('l298n_motor_driver', anonymous=True)
    driver = L298NMotorDriver()

    rospy.on_shutdown(driver.cleanup)
    rospy.spin()
