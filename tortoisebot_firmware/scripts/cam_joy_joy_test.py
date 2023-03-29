#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import rospy
from tortoisebot_firmware.msg import PtzCamera
import sys


rospy.init_node('joystick_remapper', anonymous=True)

# Define the publisher to the remapped topic
remapped_pub = rospy.Publisher('joy_remapped', Joy, queue_size=10)
pub = rospy.Publisher('PtzCamera_topic', PtzCamera, queue_size=10)

def joy_callback(data):
    # Create a new Joy message to store the remapped values
    remapped_msg = Joy()
    PtzCam = PtzCamera()

    # Remap the left joystick x-axis value to range [0, 18000]
    remapped_msg.axes = [(data.axes[0] + 1) / 2 * 17000, (data.axes[1]+ 1) / 2 * 17000]

    print(remapped_msg.axes[0],remapped_msg.axes[1])


    if (remapped_msg.axes[0] >= 17000 ):
            PtzCam.zoom   = 17000 # zoom max = 18000 ,  zoom min = 0
    elif (remapped_msg.axes[0] <= 0):
        PtzCam.zoom   = 100
    else :
        PtzCam.zoom   = remapped_msg.axes[0]

    if (remapped_msg.axes[1] >= 17000 ):
            PtzCam.focus   = 17000 # zoom max = 18000 ,  zoom min = 0
    elif (remapped_msg.axes[1] <= 0):
        PtzCam.focus   = 0
    else :
        PtzCam.focus   = remapped_msg.axes[1] # focus max = 18000  focus min = 0
            
    # PtzCam.roll = roll
    # PtzCam.pitch  = pitch
    # PtzCam.yaw    = yaw
    rospy.loginfo("published values are \n")
    rospy.loginfo(PtzCam)
    pub.publish(PtzCam)

    # Publish the remapped message
    remapped_pub.publish(remapped_msg)

# Create a subscriber to the 'joy' topic
rospy.Subscriber("joy", Joy, joy_callback)

rospy.spin()


