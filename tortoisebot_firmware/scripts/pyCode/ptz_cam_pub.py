#!/usr/bin/env python
# encoding: UTF-8

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera.array import PiRGBArray
from picamera import PiCamera
from std_msgs.msg import String
from tortoisebot_firmware.msg import PtzCamera

from sensor_msgs.msg import CompressedImage
#from picamera.array import PiRGBArray

import cv2 #sudo apt-get install python-opencv
import numpy as py
import os
import sys
import time
try:
    import picamera
    from picamera.array import PiRGBArray
except:
    sys.exit(0)

from Focuser import Focuser
from AutoFocus import AutoFocus
import curses

global image_count
image_count = 0

# Rendering status bar
# def RenderStatusBar(stdscr):
#     height, width = stdscr.getmaxyx()
#     statusbarstr = "Press 'q' to exit"
#     stdscr.attron(curses.color_pair(3))
#     stdscr.addstr(height-1, 0, statusbarstr)
#     stdscr.addstr(height-1, len(statusbarstr), " " * (width - len(statusbarstr) - 1))
#     stdscr.attroff(curses.color_pair(3))
# # Rendering description
# def RenderDescription(stdscr):
#     focus_desc      = "Focus    : Left-Right Arrow"
#     zoom_desc       = "Zoom     : Up-Down Arrow"
#     motor_x_desc    = "MotorX   : 'w'-'s' Key"
#     motor_y_desc    = "MotorY   : 'a'-'d' Key"
#     ircut_desc      = "IRCUT    : Space"
#     autofocus_desc  = "Autofocus: Enter"
#     snapshot_desc   = "Snapshot : 'c' Key"

#     desc_y = 1
    
#     stdscr.addstr(desc_y + 1, 0, focus_desc, curses.color_pair(1))
#     stdscr.addstr(desc_y + 2, 0, zoom_desc, curses.color_pair(1))
#     stdscr.addstr(desc_y + 3, 0, motor_x_desc, curses.color_pair(1))
#     stdscr.addstr(desc_y + 4, 0, motor_y_desc, curses.color_pair(1))
#     stdscr.addstr(desc_y + 5, 0, ircut_desc, curses.color_pair(1))
#     stdscr.addstr(desc_y + 6, 0, autofocus_desc, curses.color_pair(1))
#     stdscr.addstr(desc_y + 7, 0, snapshot_desc, curses.color_pair(1))
# # Rendering  middle text
# def RenderMiddleText(stdscr,k,focuser):
#     # get height and width of the window.
#     height, width = stdscr.getmaxyx()
#     # Declaration of strings
#     title = "Arducam Controller"[:width-1]
#     subtitle = ""[:width-1]
#     keystr = "Last key pressed: {}".format(k)[:width-1]
    
    
#     # Obtain device infomation
#     focus_value = "Focus    : {}".format(focuser.get(Focuser.OPT_FOCUS))[:width-1]
#     zoom_value  = "Zoom     : {}".format(focuser.get(Focuser.OPT_ZOOM))[:width-1]
#     motor_x_val = "MotorX   : {}".format(focuser.get(Focuser.OPT_MOTOR_X))[:width-1]
#     motor_y_val = "MotorY   : {}".format(focuser.get(Focuser.OPT_MOTOR_Y))[:width-1]
#     ircut_val   = "IRCUT    : {}".format(focuser.get(Focuser.OPT_IRCUT))[:width-1]
    
#     if k == 0:
#         keystr = "No key press detected..."[:width-1]

#     # Centering calculations
#     start_x_title = int((width // 2) - (len(title) // 2) - len(title) % 2)
#     start_x_subtitle = int((width // 2) - (len(subtitle) // 2) - len(subtitle) % 2)
#     start_x_keystr = int((width // 2) - (len(keystr) // 2) - len(keystr) % 2)
#     start_x_device_info = int((width // 2) - (len("Focus    : 00000") // 2) - len("Focus    : 00000") % 2)
#     start_y = int((height // 2) - 6)
    
#     # Turning on attributes for title
#     stdscr.attron(curses.color_pair(2))
#     stdscr.attron(curses.A_BOLD)

#     # Rendering title
#     stdscr.addstr(start_y, start_x_title, title)

#     # Turning off attributes for title
#     stdscr.attroff(curses.color_pair(2))
#     stdscr.attroff(curses.A_BOLD)

#     # Print rest of text
#     stdscr.addstr(start_y + 1, start_x_subtitle, subtitle)
#     stdscr.addstr(start_y + 3, (width // 2) - 2, '-' * 4)
#     stdscr.addstr(start_y + 5, start_x_keystr, keystr)
#     # Print device info
#     stdscr.addstr(start_y + 6, start_x_device_info, focus_value)
#     stdscr.addstr(start_y + 7, start_x_device_info, zoom_value)
#     stdscr.addstr(start_y + 8, start_x_device_info, motor_x_val)
#     stdscr.addstr(start_y + 9, start_x_device_info, motor_y_val)
#     stdscr.addstr(start_y + 10, start_x_device_info, ircut_val)
# parse input key
def parseKey(k,focuser,auto_focus,camera):
    global image_count
    motor_step  = 5
    focus_step  = 100
    zoom_step   = 100
    if k == ord('s'):
        focuser.set(Focuser.OPT_MOTOR_Y,focuser.get(Focuser.OPT_MOTOR_Y) + motor_step)
    elif k == ord('w'):
        focuser.set(Focuser.OPT_MOTOR_Y,focuser.get(Focuser.OPT_MOTOR_Y) - motor_step)
    elif k == ord('d'):
        focuser.set(Focuser.OPT_MOTOR_X,focuser.get(Focuser.OPT_MOTOR_X) - motor_step)
    elif k == ord('a'):
        focuser.set(Focuser.OPT_MOTOR_X,focuser.get(Focuser.OPT_MOTOR_X) + motor_step)
    elif k == ord('r'):
        focuser.reset(Focuser.OPT_FOCUS)
        focuser.reset(Focuser.OPT_ZOOM)
    elif k == curses.KEY_DOWN:
        focuser.set(Focuser.OPT_ZOOM,focuser.get(Focuser.OPT_ZOOM) - zoom_step)
   
    elif k == curses.KEY_UP:
        focuser.set(Focuser.OPT_ZOOM,focuser.get(Focuser.OPT_ZOOM) + zoom_step)

    elif k == curses.KEY_RIGHT:
        focuser.set(Focuser.OPT_FOCUS,focuser.get(Focuser.OPT_FOCUS) + focus_step)
    elif k == curses.KEY_LEFT:
        focuser.set(Focuser.OPT_FOCUS,focuser.get(Focuser.OPT_FOCUS) - focus_step)
    elif k == 10:
        # auto_focus.startFocus()
        auto_focus.startFocus2()
        # auto_focus.auxiliaryFocusing()
        pass
    elif k == 32:
        focuser.set(Focuser.OPT_IRCUT,focuser.get(Focuser.OPT_IRCUT)^0x0001)
        pass
    elif k == ord('c'):
        #set camera resolution to 2500x1900
        camera.resolution = (2500,1900)
        #save image to file.
        camera.capture("image{}.jpg".format(image_count))
        image_count += 1
        #set camera resolution to 640x480
        camera.resolution = (640,480)


# Python curses example Written by Clay McLeod
# https://gist.github.com/claymcleod/b670285f334acd56ad1c
# def draw_menu(stdscr,camera):
#     focuser = Focuser(1)
#     auto_focus = AutoFocus(focuser,camera)
    
#     k = 0
#     cursor_x = 0
#     cursor_y = 0

#     # Clear and refresh the screen for a blank canvas
#     stdscr.clear()
#     stdscr.refresh()

#     # Start colors in curses
#     curses.start_color()
#     curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
#     curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
#     curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)

#     # Loop where k is the last character pressed
#     while (k != ord('q')):
#         # Initialization
#         stdscr.clear()
#         # Flush all input buffers. 
#         curses.flushinp()
#         # get height and width of the window.
#         height, width = stdscr.getmaxyx()

#         # parser input key
#         parseKey(k,focuser,auto_focus,camera)

#         # Rendering some text
#         whstr = "Width: {}, Height: {}".format(width, height)
#         stdscr.addstr(0, 0, whstr, curses.color_pair(1))

#         # render key description
#         RenderDescription(stdscr)
#         # render status bar
#         RenderStatusBar(stdscr)
#         # render middle text
#         RenderMiddleText(stdscr,k,focuser)
#         # Refresh the screen
#         stdscr.refresh()

#         # Wait for next input
#         k = stdscr.getch()


def camerapub():
    global camera
    rospy.init_node('camera_publisher')
    image_pub = rospy.Publisher('camera/image/compressed', CompressedImage, queue_size=1)
    bridge = CvBridge()

    # Capture frames from the camera
    while not rospy.is_shutdown():
        # Grab a frame from the camera preview
        image = np.empty((camera.resolution[1] * camera.resolution[0] * 3,), dtype=np.uint8)
        camera.capture(image, 'bgr', use_video_port=True)
        image = image.reshape((camera.resolution[1], camera.resolution[0], 3))

        # Display the image
        #cv2.imshow("Frame", image)
        cv2.waitKey(1)

        # Convert the image to a ROS message and publish it
        image_msg = bridge.cv2_to_compressed_imgmsg(image, dst_format='jpeg')

        image_pub.publish(image_msg)

    # Cleanup
    cv2.destroyAllWindows()

def  PtzCamera_callback(message):
    focuser = Focuser(1)
    global image_count
    # motor_step  = 5
    # focus_step  = 100
    # zoom_step   = 100

    rospy.loginfo("zoom = %d,focus = %d,roll= %d ,pitch = %d,yaw = %d",message.zoom,message.focus,message.roll,message.pitch,message.yaw)

 
    # if k == ord('s'): + Y
    focuser.set(Focuser.OPT_MOTOR_Y, message.pitch)
    print("y pitch value",focuser.get(Focuser.OPT_MOTOR_Y))

    # elif k == ord('w'): -Y
    #     focuser.set(Focuser.OPT_MOTOR_Y,focuser.get(Focuser.OPT_MOTOR_Y) - motor_step)
    # elif k == ord('d'): -x

    focuser.set(Focuser.OPT_MOTOR_X, message.roll)
    print("x roll value",focuser.get(Focuser.OPT_MOTOR_X))

    # elif k == ord('a'): +x
    #     focuser.set(Focuser.OPT_MOTOR_X,focuser.get(Focuser.OPT_MOTOR_X) + motor_step)
    # elif k == ord('r'):
    #     focuser.reset(Focuser.OPT_FOCUS)
    #     focuser.reset(Focuser.OPT_ZOOM)


    focuser.set(Focuser.OPT_ZOOM, message.zoom)
    print("zoom value",focuser.get(Focuser.OPT_ZOOM))
 
    # focuser.set(Focuser.OPT_ZOOM,focuser.get(zoom_step_plus)

    focuser.set(Focuser.OPT_FOCUS, message.focus)
    print("focus value",focuser.get(Focuser.OPT_FOCUS))
   
    # focuser.set(Focuser.OPT_FOCUS,focuser.get(Ffocus_step_minus)


    # elif k == 10:
    #     # auto_focus.startFocus()
    #     auto_focus.startFocus2()
    #     # auto_focus.auxiliaryFocusing()
    #     pass
    # elif k == 32:
    #     focuser.set(Focuser.OPT_IRCUT,focuser.get(Focuser.OPT_IRCUT)^0x0001)
    #     pass
    # elif k == ord('c'):
    #     #set camera resolution to 2500x1900
    #     camera.resolution = (2500,1900)
    #     #save image to file.
    #     camera.capture("image{}.jpg".format(image_count))
    #     image_count += 1
    #     #set camera resolution to 640x480
    #     camera.resolution = (640,480)


def camera_command_sub():
    #rospy.init_node('PtzCamera_subscriber_node', anonymous=True)
    rospy.Subscriber("ptzcamera_topic", PtzCamera, PtzCamera_callback)

    
def main():
    global camera
    #open camera
    camera = picamera.PiCamera()
    #open camera preview
    #camera.start_preview()

    time.sleep(0.1)

    #set camera resolution to 640x480(Small resolution for faster speeds.)
    camera.resolution = (640, 480)
    camera.framerate = 32

    # https://picamera.readthedocs.io/en/release-1.13/recipes1.html?highlight=shutter%20speed#capturing-consistent-images
    # camera.iso = 100
    # camera.shutter_speed = camera.exposure_speed
    # camera.exposure_mode = 'off'

    #curses.wrapper(draw_menu,camera)

    camera_command_sub()
    camerapub()


    camera.stop_preview()
    camera.close()

    

if __name__ == "__main__":
    main()
