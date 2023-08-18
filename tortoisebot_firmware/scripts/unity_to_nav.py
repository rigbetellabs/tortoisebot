#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client(goal_loc):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_loc.position.x
    goal.target_pose.pose.position.y = goal_loc.position.y
    goal.target_pose.pose.position.z = goal_loc.position.z
    goal.target_pose.pose.orientation.x = goal_loc.orientation.x
    goal.target_pose.pose.orientation.y = goal_loc.orientation.y
    goal.target_pose.pose.orientation.z = goal_loc.orientation.z
    goal.target_pose.pose.orientation.w = goal_loc.orientation.w


    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def callback(data):
    result = movebase_client(data)
    if result:
        rospy.loginfo("Goal execution done!")

if __name__ == '__main__':
    
    rospy.init_node('movebase_client_py')
    rospy.Subscriber("/Goal", Pose, callback)
    rospy.spin()
