#!/usr/bin/env python

# python camera_control.py _zoom:=100 _focus:=100 _roll:=100 _pitch:=100 _yaw:=100
# rosrun tortoisebot_firmware camera_control.py _zoom:=100 _focus:=100 _roll:=100 _pitch:=100 _yaw:=100

import rospy
from tortoisebot_firmware.msg import PtzCamera
import sys

def camera():

    rospy.init_node('ptzcamera_publisher_node', anonymous=True)

    zoom = rospy.get_param('~zoom', default= 100)  # 
    focus = rospy.get_param('~focus', default= 100)
    roll =  rospy.get_param('~roll', default= 100)
    pitch = rospy.get_param('~pitch', default= 100)
    yaw = rospy.get_param('~yaw', default= 100)

    #create a new publisher. 
    pub = rospy.Publisher('ptzcamera_topic', PtzCamera, queue_size=10)

    #set the loop rate
    rate = rospy.Rate(2) # 1hz
    #keep publishing until a Ctrl-C is pressed

    while not rospy.is_shutdown():
        PtzCam = PtzCamera()

        if (zoom >= 17000 ):
             PtzCam.zoom   = 17000 # zoom max = 18000 ,  zoom min = 0
        elif (zoom <= 0):
            PtzCam.zoom   = 100
        else :
            PtzCam.zoom   = zoom

        if (focus >= 17000 ):
             PtzCam.focus   = 17000 # zoom max = 18000 ,  zoom min = 0
        elif (focus <= 0):
            PtzCam.focus   = 0
        else :
            PtzCam.focus   = focus # focus max = 18000  focus min = 0
            
        PtzCam.roll = roll
        PtzCam.pitch  = pitch
        PtzCam.yaw    = yaw
        rospy.loginfo("published values are \n")
        rospy.loginfo(PtzCam)
        pub.publish(PtzCam)
        rate.sleep()



if __name__ == '__main__':
    try:
        camera()
    except rospy.ROSInterruptException:
        pass
