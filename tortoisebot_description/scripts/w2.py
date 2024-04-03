#!/usr/bin/env python3
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Float32

verbose = rospy.Publisher('checkpoints_verbose', String, queue_size=10)
timestamp = rospy.Publisher('checkpoints_verbose', Float32, queue_size=10)

def movebase_client(coords):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = coords[0]
    goal.target_pose.pose.position.y = coords[1]
    goal.target_pose.pose.orientation.z = coords[2]
    goal.target_pose.pose.orientation.w = coords[3]
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def arrow_detected(arrow):
    if waypts_dict[arrow.data] not in waypoints_list:
        waypoints_list.append(waypts_dict[arrow.data])
        waypoints_list.append(waypts_dict[arrow.data+'1'])
        timestamp.publish(rospy.get_time())
        verbose.publish()
        

waypts_dict = {'p1':(0.52,0.53,0.68,0.72),'p2':(0.29,0.92,0.97,0.20),'p3':(-0.13,0.42,0.99,0.18),'left':(-0.61,-0.04,0.98,0.18),'left1':(-0.61,-0.04,0.98,0.3),'right':(-0.65,1.04,0.99,0.028),'right1':(-0.75,0.93,-0.77,0.62)}
waypoints_list = [waypts_dict['p1'],waypts_dict['p2'],waypts_dict['p2'],waypts_dict['p3']]#,waypts_dict['right']]


if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        rospy.Subscriber('arrow_dir',String, arrow_detected)
        verbose.publish('end')
        timestamp.publish(rospy.get_time())
        while len(waypoints_list) > 0:
            result = movebase_client(waypoints_list.pop(0))
        timestamp.publish(rospy.get_time())
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

