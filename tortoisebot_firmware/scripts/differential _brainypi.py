#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rblemi.msg import Diff
import time
from math import pi

diff_msg = Diff()

motor_rpm = 60              #   max rpm of motor on full voltage 
wheel_diameter = 0.065      	#   in meters
wheel_separation = 0.17    #   in meters
max_pwm_val = 255           #   100 for Raspberry Pi , 255 for Arduino
min_pwm_val = 0            	#   Minimum PWM value that is needed for the robot to move

wheel_radius = wheel_diameter/2
circumference_of_wheel = 2 * pi * wheel_radius
max_speed = (circumference_of_wheel*motor_rpm)/60   #   m/sec
max_angle = (max_speed*2)/wheel_separation          #   rad/sec

def stop():
    global diff_pub
    global diff_msg

    diff_msg.lpwm.data = 0
    diff_msg.rpwm.data = 0
    diff_msg.ldir.data = 1
    diff_msg.rdir.data = 1
    
    #print('stopping')

    diff_pub.publish(diff_msg)
    
def wheel_vel_executer(left_speed, right_speed):
    global max_pwm_val
    global min_pwm_val
    
    global diff_pub
    global diff_msg
    
    lspeedPWM = max(min(((abs(left_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)
    rspeedPWM = max(min(((abs(right_speed)/max_speed)*max_pwm_val),max_pwm_val),min_pwm_val)

    diff_msg.lpwm.data = int(lspeedPWM)
    diff_msg.rpwm.data = int(rspeedPWM)
    
    if left_speed >= 0 :
        diff_msg.ldir.data = 0
    else :
        diff_msg.ldir.data = 1
        
    if right_speed >= 0 :
        diff_msg.rdir.data = 1
    else :
        diff_msg.rdir.data = 0

    diff_pub.publish(diff_msg)
    
def callback(data):

    # refer this for understanding the formula 
    # http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf
    
    global wheel_radius
    global wheel_separation
    
    linear_vel = data.linear.x                  # Linear Velocity of Robot
    angular_vel = data.angular.z                # Angular Velocity of Robot
    #print(str(linear)+"\t"+str(angular))
    
    VrplusVl  = 2 * linear_vel
    VrminusVl = angular_vel * wheel_separation
    
    right_vel = ( VrplusVl + VrminusVl ) / 2      # right wheel velocity along the ground
    left_vel  = VrplusVl - right_vel              # left wheel velocity along the ground
    
    #print (str(left_vel)+"\t"+str(right_vel))
    
    if (left_vel == 0.0 and right_vel == 0.0):
        stop()
    else:
        wheel_vel_executer(left_vel, right_vel)
        
def listener():
    
    global diff_pub
    
    rospy.init_node('cmdvel_listener', anonymous=False)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    diff_pub = rospy.Publisher('diff', Diff, queue_size = 10)
    rospy.spin()

if __name__== '__main__':
    print('Tortoisebot Differential Drive Initialized with following Params-')
    print('Motor Max RPM:\t'+str(motor_rpm)+' RPM')
    print('Wheel Diameter:\t'+str(wheel_diameter)+' m')
    print('Wheel Separation:\t'+str(wheel_separation)+' m')
    print('Robot Max Speed:\t'+str(max_speed)+' m/sec')
    print('Max Angular Speed:\t'+str(max_angle)+' rad/sec')
    listener()
