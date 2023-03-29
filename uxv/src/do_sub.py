#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
import time

d1_pin = 19
d2_pin = 13
d3_pin = 6
d4_pin = 5
d5_pin = 21 
d6_pin = 20
d7_pin = 16
d8_pin = 26

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(d1_pin, GPIO.OUT)
GPIO.setup(d2_pin, GPIO.OUT)
GPIO.setup(d3_pin, GPIO.OUT)
GPIO.setup(d4_pin, GPIO.OUT)
GPIO.setup(d5_pin, GPIO.OUT)
GPIO.setup(d6_pin, GPIO.OUT)
GPIO.setup(d7_pin, GPIO.OUT)
GPIO.setup(d8_pin, GPIO.OUT)

GPIO.output(d1_pin, GPIO.LOW)
GPIO.output(d2_pin, GPIO.LOW)
GPIO.output(d3_pin, GPIO.LOW)
GPIO.output(d4_pin, GPIO.LOW)
GPIO.output(d5_pin, GPIO.LOW)
GPIO.output(d6_pin, GPIO.LOW)
GPIO.output(d7_pin, GPIO.LOW)
GPIO.output(d8_pin, GPIO.LOW)

def d1(data):
    if data.data == 1:
        GPIO.output(d1_pin, GPIO.HIGH)
    else:
        GPIO.output(d1_pin, GPIO.LOW)
        
def d2(data):
    if data.data == 1:
        GPIO.output(d2_pin, GPIO.HIGH)
    else:
        GPIO.output(d2_pin, GPIO.LOW)
        
def d3(data):
    if data.data == 1:
        GPIO.output(d3_pin, GPIO.HIGH)
    else:
        GPIO.output(d3_pin, GPIO.LOW)
        
def d4(data):
    if data.data == 1:
        GPIO.output(d4_pin, GPIO.HIGH)
    else:
        GPIO.output(d4_pin, GPIO.LOW)

def d5(data):
    if data.data == 1:
        GPIO.output(d5_pin, GPIO.HIGH)
    else:
        GPIO.output(d5_pin, GPIO.LOW)
        
def d6(data):
    if data.data == 1:
        GPIO.output(d6_pin, GPIO.HIGH)
    else:
        GPIO.output(d6_pin, GPIO.LOW)
        
def d7(data):
    if data.data == 1:
        GPIO.output(d7_pin, GPIO.HIGH)
    else:
        GPIO.output(d7_pin, GPIO.LOW)
        
def d8(data):
    if data.data == 1:
        GPIO.output(d8_pin, GPIO.HIGH)
    else:
        GPIO.output(d8_pin, GPIO.LOW)
        
    
def do_listener():
    rospy.init_node('do_listener', anonymous=False)
    rospy.Subscriber("/do/1", Bool, d1)
    rospy.Subscriber("/do/2", Bool, d2)
    rospy.Subscriber("/do/3", Bool, d3)
    rospy.Subscriber("/do/4", Bool, d4)
    rospy.Subscriber("/do/5", Bool, d5)
    rospy.Subscriber("/do/6", Bool, d6)
    rospy.Subscriber("/do/7", Bool, d7)
    rospy.Subscriber("/do/8", Bool, d8)
    rospy.spin()

if __name__== '__main__':
    print('Digital Outputs Initialized')
    do_listener()
