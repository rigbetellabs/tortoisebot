#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class aligner:
    def __init__(self):
        self.offset = 0.0
        self.straight_line = 0.0
        rospy.Subscriber('docking_offset',String,self.callback_offset)
        rospy.Subscriber('scan',LaserScan,self.callback_laser)
    def pub_loc(self):
        print(self.straight_line)
    def callback_offset(self,data):
        self.offset = data.split(';')
        self.offset[0] = eval(self.offset(0))
        self.offset[1] = eval(self.offset(1))
        print(data)
    def callback_laser(self,data):
        self.straight_line = data.ranges[360]
        
        

rospy.init_node('result', anonymous=True)
aligner()
rospy.spin()
