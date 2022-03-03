#! /usr/bin/env python3
import apriltag
import cv2 as cv
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge

pub = rospy.Publisher('docking_offset', String , queue_size=10)
rospy.init_node('offset_calc', anonymous=True)
rate = rospy.Rate(10)
   
def img_converter(img):
    bridge = CvBridge()
    
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
    return(cv_image)


def result(coord,size_):
     # 10hz
    while not (rospy.is_shutdown()):
        if coord is not None:
            centre = "left %s" % rospy.get_time()
            rospy.loginfo(coord)
            rospy.loginfo(size_)
            data = str(str(coord)+';'+str(size))
            pub.publish(data)
            rate.sleep()
        else:
            pass


def april(img):

    print("Loading image...")
    image = img_converter(img)#subscribe to /camera/image_raw/
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    img_shape = (image.shape[1]//2,image.shape[0]//2)
    print("Detecting AprilTags...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    print("{} total AprilTags detected".format(len(results)))#print the no.of apriltags detected

    # loop over the AprilTag detection results
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv.line(image, ptA, ptB, (0, 255, 0), 2)
        cv.line(image, ptB, ptC, (0, 255, 0), 2)
        cv.line(image, ptC, ptD, (0, 255, 0), 2)
        cv.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv.putText(image, tagFamily, (ptA[0], ptA[1] - 15),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print("[INFO] tag family: {}".format(tagFamily))
    # show the output image after AprilTag detection
        result(coord,size_)


if __name__ == '__main__':
    try:
        rospy.Subscriber("/camera/image_raw",Image,april)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
