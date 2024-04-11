#!/usr/bin/env python3

import transforms3d

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class GoToGoal(Node):

    def __init__(self):
        super().__init__('go_to_goal')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.master_callback)

        self.waypoints = [[0.0, 0.0, 2.3562], [0.0, 1.0, 0.7854], [1.0, 1.0, -0.7854], [1.0, 0.0, -2.3562]]
        self.counter = 0

        self.reached_goal = True


    def master_callback(self):

        if self.reached_goal:
            self.counter = (self.counter + 1) % 4
            self.get_logger().info(f'Sending Goal number: {self.counter}')
            self.send_goal()

    def send_goal(self):

        self.reached_goal = False

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.waypoints[self.counter][0]
        goal.pose.position.y = self.waypoints[self.counter][1]
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        
        goal.pose.orientation.w, _, _, goal.pose.orientation.z = transforms3d.euler.euler2quat(0 , 0, self.waypoints[self.counter][2])


        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal Accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.reached_goal = True
        result = future.result().result
        self.get_logger().info(f"Result -> SUCCESS")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback -> Distance remaining: {feedback.distance_remaining}", throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)

    gotogoal = GoToGoal()

    rclpy.spin(gotogoal)


if __name__ == '__main__':
    main()