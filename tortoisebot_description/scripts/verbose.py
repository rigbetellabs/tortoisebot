#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

time = [0.0]

def callback_time(data):
    time.append(data)
    print(data,time)

def callback_str(data):
    if data.data == 'left':
        rospy.loginfo(rospy.get_caller_id() + "Robot takes a left, now heading towards Home docking station.")
    elif data.data == 'right':
        rospy.loginfo(rospy.get_caller_id() + "Robot takes a right, now heading towards Parking docking station.")
    elif data.data == 'start':
        rospy.loginfo(rospy.get_caller_id() + "Tortoise robot has begun its journey.")
    elif data.data == 'end':
        rospy.loginfo(rospy.get_caller_id() + "Tortoise robot has ended its journey.")
        duration()
        

def info():
    rospy.init_node('info', anonymous=True)
    rospy.Subscriber("checkpoint_timestamp", Float32, callback_str)
    rospy.Subscriber("checkpoint_verbose", String, callback_time)

def duration():
    print("Total Duration for the robot to reach the intersection:", "{:.3f}".format(time[2]-time[1]), "seconds")
    print("Total Duration for the robot to dock:", "{:.3f}".format(time[3]-time[1]), "seconds")

if __name__ == '__main__':
    info()
    rospy.spin()

