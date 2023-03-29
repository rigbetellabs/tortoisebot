#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

class CameraPublisher:
    def __init__(self):
        self.publisher = rospy.Publisher('camera/image_raw', Image, queue_size=10)
        self.bridge = CvBridge()
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 32
        self.raw_capture = PiRGBArray(self.camera, size=(640, 480))
        time.sleep(0.1)

    def publish_image(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher.publish(ros_image)

    def run(self):
        for frame in self.camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True):
            image = frame.array

            # Process the image here using OpenCV
            gray_frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Publish the image in ROS
            self.publish_image(image)

            # Clear the stream for the next frame
            self.raw_capture.truncate(0)


def main():
    rospy.init_node('camera_publisher', anonymous=True)
    camera_publisher = CameraPublisher()
    camera_publisher.run()

if __name__ == '__main__':
    main()

