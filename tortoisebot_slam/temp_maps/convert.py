#!/usr/bin/env python3
# license removed for brevity

import os
from PIL import Image as IM
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def talker():
    
    pub = rospy.Publisher('image_converted', Image, queue_size=10)
    rospy.init_node('image_converter', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    
    while not rospy.is_shutdown():
        im1 = IM.open(r'/home/shubhu/catkin_ws/src/tortoisebot/tortoisebot_slam/temp_maps/rbl_map.pgm')
        img_file = im1.save(r'rbl_map.png')
        # rospy.loginfo(im1.image)
        cv_image = cv2.imread('rbl_map.png',0)
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        # msg = (cv_image, "bgr8")
        pub.publish(msg)
        # rospy.loginfo(msg)
        print(type(msg))
        
        rate.sleep()
    

if __name__ == '__main__':
    talker()