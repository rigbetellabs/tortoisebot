#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from rblemi.msg import Diff
import time
from math import pi

def move():
	rospy.init_node('move', anonymous = True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	move_cmd=Twist()

	#STOP
	move_cmd.linear.x=0
	move_cmd.angular.z=0
	pub.publish(move_cmd)
	time.sleep(1)
	#FWD
	move_cmd.linear.x=0.2
        move_cmd.angular.z=0
        pub.publish(move_cmd)
        time.sleep(1)
	#STOP
        move_cmd.linear.x=0
        move_cmd.angular.z=0
        pub.publish(move_cmd)
        time.sleep(1)

	#TURN
        move_cmd.linear.x=0
        move_cmd.angular.z=2.0
        pub.publish(move_cmd)
        time.sleep(1)
        #STOP
        move_cmd.linear.x=0
        move_cmd.angular.z=0
        pub.publish(move_cmd)
        time.sleep(1)


if __name__ == '__main__':

	try:
		move()
	except rospy.ROSInterruptException:
		pass

