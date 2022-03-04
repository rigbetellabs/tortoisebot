#!/usr/bin/env python
import rospy 
from geometry_msgs.msg import Twist

def move():
	rospy.init_node('move')
	pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
	rate = rospy.Rate(1)

	v = Twist()
	rate.sleep()
	while True:
	
		v.linear.x = 1 #m/s
		pub.publish(v)
		rate.sleep()
		rospy.loginfo(v)
		break
	v.linear.x = 0
	pub.publish(v)
	rate.sleep()
	rospy.loginfo(v) 

if __name__ == '__main__':
	move()