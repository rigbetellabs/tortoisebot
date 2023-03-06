#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid as OG

def callback(data):
     
    # rospy.loginfo((data.data))
    map_data = data.data
    map_list = list(map_data)

    # map_list[len(map_list)-1] = 255

    for i in range(len(map_list)):
        if map_list[i] <= 75 :
            map_list[i] = 0
        else:
            map_list[i] = 127

    data.data = tuple(map_list)

    #print(data.data)

    map_pub.publish(data)

if __name__ == '__main__':
    rospy.init_node('mapUnity')
    map_pub = rospy.Publisher('/map_unity', OG, queue_size = 10)
    rospy.Subscriber('/map', OG, callback)
    
    rospy.spin()