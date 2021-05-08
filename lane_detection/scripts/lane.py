#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from velocity import Robot
from geometry_msgs.msg import Twist
import sys

class Lane(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.camera_callback)
        self.moverosbots_object = Robot()
        self.fault_checker =0 

    def camera_callback(self,data):
        
        try:
            
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        

        height, width, channels = cv_image.shape
        rows_to_watch = 700
        top_trunc = 1*height / 2 
        bot_trunc = top_trunc + rows_to_watch 
        crop_img = cv_image[top_trunc:bot_trunc, 0:width-200]
        

        h_upper = cv2.getTrackbarPos('Hu','image')
        s_upper = cv2.getTrackbarPos('Su','image')
        v_upper = cv2.getTrackbarPos('Vu','image')
        h_lower = cv2.getTrackbarPos('Hl','image')
        s_lower = cv2.getTrackbarPos('Sl','image')
        v_lower = cv2.getTrackbarPos('Vl','image')
        
        
        
        
        
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV).astype(np.float)
        
        
        lower_yellow = np.array([0,0,70])
        upper_yellow = np.array([0,0,137])

       
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        number_of_white_pix = np.sum(mask == 255)
        

        height1, width1, channels1 = crop_img.shape
        
        
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height1/2, width1/2

        
        
        
       
        res = cv2.bitwise_and(crop_img,crop_img, mask= mask)
        
        
        cv2.circle(res,(int(cx), int(cy)), 20,(0,0,255),-1)

        cv2.imshow("Original", cv_image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("MASK", mask)
        cv2.imshow("RES", res)
        
        cv2.waitKey(1)
        
        if(number_of_white_pix > 32000 ):
            self.fault_checker = self.fault_checker + 1

            if(number_of_white_pix > 32000 and self.fault_checker>20):
            

                rospy.loginfo("No of white pixel ===>"+str(number_of_white_pix))
                rospy.signal_shutdown("reason")

        else:
            error_x = cx - width1 / 2;
            angular_z = -error_x / 100;
            rospy.loginfo("ANGULAR VALUE SENT===>"+str(angular_z))
            twist_object = Twist();
            twist_object.linear.x = 0.2;
            twist_object.angular.z = -error_x / 100;
            # Robot starts moving
            self.moverosbots_object.move_robot(twist_object)
        
    def clean_up(self):
        self.moverosbots_object.clean_class()
        cv2.destroyAllWindows()

    def nothing(self,x):
        pass
        
        

def main():
    rospy.init_node('line_following_node', anonymous=True)
    cv2.namedWindow('image')
    """
    cv2.createTrackbar('Hu','image',0,255,nothing)
    cv2.createTrackbar('Su','image',0,255,nothing)
    cv2.createTrackbar('Vu','image',0,255,nothing)
    cv2.createTrackbar('Hl','image',0,255,nothing)
    cv2.createTrackbar('Sl','image',0,255,nothing)
    cv2.createTrackbar('Vl','image',0,255,nothing)
    """
    
    
    line_follower_object = Lane()
   
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        
        
        line_follower_object.clean_up()
        rospy.loginfo("shutdown time!")
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        rate.sleep()

def nothing(x):
    pass

    
    
if __name__ == '__main__':
    main()