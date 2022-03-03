#! /usr/bin/env python3
import rospy
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from detection import side
from cv_bridge import CvBridge

pub = rospy.Publisher('arrow_dir', String, queue_size=10)

# Function to increase brightness of the cv Image Object
def increase_brightness(img,value):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv.merge((h, s, v))
    img = cv.cvtColor(final_hsv, cv.COLOR_HSV2BGR)
    return img

# Function to detect the arrow and classify its direction
def detect(Image):
    bridge = CvBridge()
    global detected
    image = bridge.imgmsg_to_cv2(Image, desired_encoding='passthrough')
    #image = increase_brightness(cv_image, value = 150)
    image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    _, threshold = cv.threshold(image, 50, 255, cv.THRESH_BINARY)
    contours,_=cv.findContours(threshold, cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    #print(contours)
    
    for cnt in contours :
        area = cv.contourArea(cnt)
        if area > 400: 
            #print(area)
            approx = cv.approxPolyDP(cnt, 0.009 * cv.arcLength(cnt, True), True)
            if(len(approx) == 7): 
                cv.drawContours(image, [approx], 0, (0, 255, 0), 5)
                min_o = []
                for i in approx:
                    min_o.append(i[0][0])
                min_o.sort()
                if ((min_o[1] - min_o[0] > 5) and (min_o[-1] - min_o[-2] < 5)):
                    #print ("go Left")
                    pub.publish('left')

                elif ((min_o[1] - min_o[0] < 5) and (min_o[-1] - min_o[-2] > 5)):
                    #print ("go Right")
                    pub.publish('right')

image_message = rospy.Subscriber("/camera/image_raw",Image,detect)


if __name__ == '__main__':
    rospy.init_node('arrow_detector', anonymous=True)
    rospy.spin()

