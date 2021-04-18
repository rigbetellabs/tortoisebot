#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

leftEn = 12			#	Purple
rightEn = 13		#	Red

leftBackward = 5	#	Blue
leftForward = 6		#	Green
rightForward = 16	#	Yellow
rightBackward = 20	#	Orange

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(leftEn, GPIO.OUT)
GPIO.setup(rightEn, GPIO.OUT)
GPIO.setup(leftForward, GPIO.OUT)
GPIO.setup(leftBackward, GPIO.OUT)
GPIO.setup(rightForward, GPIO.OUT)
GPIO.setup(rightBackward, GPIO.OUT)

pwmL = GPIO.PWM(leftEn, 100)
pwmL.start(0)
pwmR = GPIO.PWM(rightEn, 100)
pwmR.start(0)

def stop():
    print('stopping')
    pwmL.ChangeDutyCycle(0)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.HIGH)
    pwmR.ChangeDutyCycle(0)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.HIGH)

def forward():
    print('going forward')
    pwmL.ChangeDutyCycle(100)
    pwmR.ChangeDutyCycle(100)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.LOW)
    GPIO.output(rightBackward, GPIO.LOW)

def backward():
    print('going backward')
    pwmL.ChangeDutyCycle(100)
    pwmR.ChangeDutyCycle(100)
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.LOW)
    GPIO.output(leftBackward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.HIGH)

def left(turn):
    print('turning left')
    pwmL.ChangeDutyCycle(100)
    pwmR.ChangeDutyCycle(100)
    GPIO.output(leftForward, GPIO.LOW)
    GPIO.output(leftBackward, GPIO.HIGH)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.LOW)

def right(turn):
    print('turning right')
    pwmL.ChangeDutyCycle(100)
    pwmR.ChangeDutyCycle(100)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.LOW)
    GPIO.output(rightForward, GPIO.LOW)
    GPIO.output(rightBackward, GPIO.HIGH)

def callback(data):
    linear = data.linear.x
    angular = data.angular.z
    #print(str(linear)+"\t"+str(angular))
    if (linear == 0.0 and angular == 0.0):
        stop()
    elif (linear > 0.0 and angular == 0.0):
        forward()
    elif (linear < 0.0 and angular == 0.0):
        backward()
    elif (linear == 0.0 and angular > 0.0):
        left()
    elif (linear == 0.0 and angular < 0.0):
        right()
    else:
        stop()
        
def listener():
    rospy.init_node('cmdvel_listener', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__== '__main__':
    print('Tortoisebot Differential Drive Initialized')
    listener()
