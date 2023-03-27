#!/usr/bin/env python
import rospy
from tortoisebot_firmware.msg import PtzCamera


def PtzCamera_callback(message):
    print("inside cb")
    rospy.loginfo("zoom = %d,focous = %d,roll = %d,pitch = %d,yaw = %d",
        message.zoom,message.focous,message.roll,
        message.pitch,message.yaw)
    # print(message.zoom)
    # print(message.focous)
    # print(message.pitch)
    # print(message.yaw)

def camera_subscriber():
    print("inside sub")
    rospy.init_node('PtzCamera_subscriber_node', anonymous=True)

    rospy.Subscriber("PtzCamera_topic", PtzCamera, PtzCamera_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        camera_subscriber()
    except rospy.ROSInterruptException:
        pass