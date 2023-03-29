#!/usr/bin/env python3
from __future__ import division
import Adafruit_PCA9685
import time
import rospy
from std_msgs.msg import Int16

pwm = Adafruit_PCA9685.PCA9685()

yaw_servo_pin = 0
pitch_servo_pin = 1

yaw_servo_angle = 90
yaw_servo_error = 3
yaw_servo_min = 127
yaw_servo_max = 600
yaw_pwm_diff = yaw_servo_max - yaw_servo_min


pitch_servo_angle = 90
pitch_servo_error = 0
pitch_servo_min = 200
pitch_servo_max = 520
pitch_pwm_diff = pitch_servo_max - pitch_servo_min

pwm.set_pwm_freq(60)

def yaw(ang):
    yaw_servo_pwm = yaw_servo_min+(((ang.data+yaw_servo_error)/180)*yaw_pwm_diff)
    pwm.set_pwm( yaw_servo_pin, 0, int(yaw_servo_pwm) )
    
def pitch(ang):
    pitch_servo_pwm = pitch_servo_min+(((ang.data+pitch_servo_error)/180)*pitch_pwm_diff)
    pwm.set_pwm( pitch_servo_pin, 0, int(pitch_servo_pwm) )
    
def servo_listener():
    rospy.init_node('servo_listener', anonymous=False)
    rospy.Subscriber("/servo/yaw", Int16, yaw)
    rospy.Subscriber("/servo/pitch", Int16, pitch)
    rospy.spin()

if __name__== '__main__':
    print('Servo Initialized')
    servo_listener()
