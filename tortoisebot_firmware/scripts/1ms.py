#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist

def move():
    rclpy.init()
    node = rclpy.create_node('move')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    rate = node.create_rate(1)

    v = Twist()
    rate.sleep()
    while True:
        v.linear.x = 1  # m/s
        pub.publish(v)
        rate.sleep()
        node.get_logger().info(str(v))
        break

    v.linear.x = 0
    pub.publish(v)
    rate.sleep()
    node.get_logger().info(str(v))

    rclpy.shutdown()

if __name__ == '__main__':
    move()
