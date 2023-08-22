#!/usr/bin/env python3

# This scripts takes the odom from TF published by Cartographer and publishes it as an individual topic. Only required when used with real robot.

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import tf

#Node to handle calculating and publishing odometry
class odometry_publisher():
	def __init__(self):
		#initialize the node
		rospy.init_node('odom_publisher')

		# Transform listener
		self.listener = tf.TransformListener()

		# Odometry publisher
		self.odom_publisher = rospy.Publisher('odom',Odometry,queue_size=1)

		#Sleep rate
		self.sleep_freq = 10.0
		self.rate=rospy.Rate(self.sleep_freq)

		# Required Constants
		self.old_pose = self.get_init_pose()
		print(self.old_pose)


	def get_init_pose(self):
		while True:
			try:
				old_pose = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
				break
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
		return old_pose


	def calc_velocity(self,new_pose):
		#print(new_pose)
		o = Odometry()
		o.header.frame_id="odom"
		o.child_frame_id="base_link"
		"""
		=======THIS IS WHERE CONVERSION HAPPENS=======
		"""
		o.twist.twist.linear.x = (new_pose[0][0] - self.old_pose[0][0])/self.sleep_freq
		o.twist.twist.linear.y = (new_pose[0][1] - self.old_pose[0][1])/self.sleep_freq
		o.twist.twist.linear.z = (new_pose[0][2] - self.old_pose[0][2])/self.sleep_freq
		o.twist.twist.angular.x = (new_pose[1][0] - self.old_pose[1][0])/self.sleep_freq
		o.twist.twist.angular.y = (new_pose[1][1] - self.old_pose[1][1])/self.sleep_freq
		o.twist.twist.angular.z = (new_pose[1][2] - self.old_pose[1][2])/self.sleep_freq
		o.pose.pose.position.x = new_pose[0][0]
		o.pose.pose.position.y = new_pose[0][1]
		o.pose.pose.position.z = new_pose[0][2]
		o.pose.pose.orientation.x = new_pose[1][0]
		o.pose.pose.orientation.y = new_pose[1][1]
		o.pose.pose.orientation.z = new_pose[1][2]
		o.pose.pose.orientation.w = new_pose[1][3]
		"""
		===============================================
		"""
		self.old_pose = new_pose
		return o


	def run(self):
		while not rospy.is_shutdown():
			try:
				#Get transform between frames
				new_pose = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
				#Process the transform obtained
				odom_data = self.calc_velocity(new_pose)
				#Publish odometry
				self.odom_publisher.publish(odom_data)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			self.rate.sleep()


if __name__ == '__main__':
	odometry_publisher().run()
