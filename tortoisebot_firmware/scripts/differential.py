#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
import RPi.GPIO as GPIO
import time
from math import pi

leftEn = 13         #   Purple
rightEn = 12        #   Red

leftBackward = 5    #   Blue
leftForward = 6     #   Green
rightForward = 16   #   Yellow
rightBackward = 20  #   Orange

motor_rpm = 60              #   max rpm of motor on full voltage 
wheel_diameter = 0.065      #   in meters
wheel_separation = 0.17     #   in meters
max_pwm_val = 100           #   100 for Raspberry Pi , 255 for Arduino
min_pwm_val = 15           #   Minimum PWM value that is needed for the robot to move

wheel_radius = wheel_diameter/2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel*motor_rpm)/60   #   m/sec
lPWM=Int32()
rPWM=Int32()
lDIR=Bool()
rDIR=Bool()

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

def stop(self):
    global lPWM, rPWM, lDIR, rDIR
    
    #print('stopping')
    pwmL.ChangeDutyCycle(0)
    GPIO.output(leftForward, GPIO.HIGH)
    GPIO.output(leftBackward, GPIO.HIGH)
    pwmR.ChangeDutyCycle(0)
    GPIO.output(rightForward, GPIO.HIGH)
    GPIO.output(rightBackward, GPIO.HIGH)
    lPWM.data=0
    rPWM.data=0
    lDIR.data=True
    rDIR.data=True
    self.lpwm_pub.publish(lPWM)
    self.rpwm_pub.publish(rPWM)
    self.ldir_pub.publish(lDIR)
    self.rdir_pub.publish(rDIR)
    
def wheel_vel_executer(self, left_speed, right_speed):
    global max_pwm_val
    global min_pwm_val
    global lPWM, rPWM, lDIR, rDIR

    lspeedPWM = max(min(((abs(left_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    rspeedPWM = max(min(((abs(right_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    lPWM.data=int(lspeedPWM)
    rPWM.data=int(rspeedPWM)
    pwmL.ChangeDutyCycle(lspeedPWM)
    pwmR.ChangeDutyCycle(rspeedPWM)
    
    self.lpwm_pub.publish(lPWM)
    self.rpwm_pub.publish(rPWM)
    
    if left_speed >= 0 :
        GPIO.output(leftForward, GPIO.HIGH)
        GPIO.output(leftBackward, GPIO.LOW)
        lDIR.data=True
        self.ldir_pub.publish(lDIR)
    else :
        GPIO.output(leftForward, GPIO.LOW)
        GPIO.output(leftBackward, GPIO.HIGH)
        lDIR.data=False
        self.ldir_pub.publish(lDIR)
        
    if right_speed >= 0 :
        GPIO.output(rightForward, GPIO.HIGH)
        GPIO.output(rightBackward, GPIO.LOW)
        rDIR.data=True
        self.rdir_pub.publish(rDIR)
    else :
        GPIO.output(rightForward, GPIO.LOW)
        GPIO.output(rightBackward, GPIO.HIGH)
        rDIR.data=False
        self.rdir_pub.publish(rDIR)

class Differential(Node):
    def __init__(self):
        super().__init__('differential')

        self.vel_subscription= self.create_subscription(Twist,"cmd_vel",self.callback,10)
        self.vel_subscription
        self.lpwm_pub = self.create_publisher(Int32, 'lpwm', 10)
        self.rpwm_pub = self.create_publisher(Int32, 'rpwm', 10)
        self.ldir_pub = self.create_publisher(Bool, 'ldir', 10)
        self.rdir_pub = self.create_publisher(Bool, 'rdir', 10)
    
    def callback(self, data):
        
        global wheel_radius
        global wheel_separation
        
        linear_vel = data.linear.x                  # Linear Velocity of Robot
        angular_vel = data.angular.z                # Angular Velocity of Robot


        VrplusVl  = 2 * linear_vel
        VrminusVl = angular_vel * wheel_separation
        
        right_vel = ( VrplusVl + VrminusVl ) / 2      # right wheel velocity along the ground
        left_vel  = VrplusVl - right_vel              # left wheel velocity along the ground
        
        # print (str(left_vel)+"\t"+str(right_vel))
        
        if (left_vel == 0.0 and right_vel == 0.0):
            stop(self)
        else:
            wheel_vel_executer(self, left_vel, right_vel)

def main(args=None):
    
  rclpy.init(args=args)
  differential_drive = Differential()
  rclpy.spin(differential_drive)
  differential_drive.destroy_node()
  rclpy.shutdown()
   
if __name__ == '__main__':
    print('Tortoisebot Differential Drive Initialized with following Params-')
    print('Motor Max RPM:\t'+str(motor_rpm)+' RPM')
    print('Wheel Diameter:\t'+str(wheel_diameter)+' m')
    print('Wheel Separation:\t'+str(wheel_separation)+' m')
    print('Robot Max Speed:\t'+str(max_speed)+' m/sec')
    main()
