import rospy
from sensor_msgs.msg import Joy
from tortoisebot_firmware.msg import PtzCamera
import sys

zoom_count = 100
focous_count = 100
pub = rospy.Publisher('PtzCamera_topic', PtzCamera, queue_size=10)


def joy_callback(data):
    global zoom_count
    global focous_count

    PtzCam = PtzCamera()

    if data.buttons[0] == 1:  # Change index to match the button you want to use
        zoom_count += 100
        rospy.loginfo("Count incremented to %d", zoom_count)
        PtzCam.zoom   = zoom_count
        pub.publish(PtzCam)

    elif data.buttons[0] == 0:  # Change index to match the button you want to use
        #zoom_count -= 100
        rospy.loginfo("Count incremented to %d", zoom_count)
        PtzCam.zoom   = zoom_count
        pub.publish(PtzCam)

    if data.buttons[3] == 1:  # Change index to match the button you want to use
        zoom_count -= 100
        rospy.loginfo("Count incremented to %d", zoom_count)
        PtzCam.zoom   = zoom_count
        pub.publish(PtzCam)

    elif data.buttons[3] == 0:  # Change index to match the button you want to use
        #zoom_count -= 100
        rospy.loginfo("Count incremented to %d", zoom_count)
        PtzCam.zoom   = zoom_count
        pub.publish(PtzCam)

    
    if data.buttons[2] == 1:  # Change index to match the button you want to use
        focous_count += 100
        rospy.loginfo("Count incremented to %d", focous_count)
        PtzCam.focous = focous_count
        pub.publish(PtzCam)

    elif data.buttons[2] == 0:
        #zoom_count -= 100
        rospy.loginfo("Count incremented to %d", zoom_count)
        PtzCam.focous   = focous_count
        pub.publish(PtzCam)

    if data.buttons[1] == 1:  # Change index to match the button you want to use
        focous_count -= 100
        rospy.loginfo("Count incremented to %d", focous_count)
        PtzCam.focous = focous_count
        pub.publish(PtzCam)

    elif data.buttons[1] == 0:  # Change index to match the button you want to use
        #zoom_count -= 100
        rospy.loginfo("Count incremented to %d", zoom_count)
        PtzCam.focous   = focous_count
        pub.publish(PtzCam)

    rospy.loginfo("published values are \n")
    rospy.loginfo(PtzCam)
    




def joystick_listener():
    rospy.init_node('joystick_listener')
    rospy.Subscriber("joy", Joy, joy_callback)
    rospy.spin()

if __name__ == '__main__':
    joystick_listener()
