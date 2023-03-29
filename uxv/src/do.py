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
    
