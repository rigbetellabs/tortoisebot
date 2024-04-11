#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose, FollowWaypoints
from geometry_msgs.msg import PoseStamped


class GoToGoal(Node):
    """
    A class to send a sequence of waypoints to the navigation stack and monitor feedback.

    Attributes:
        _action_client: The ActionClient to send FollowWaypoints goals.
        waypoints (list): A list to store the sequence of waypoints.
        goal (PoseStamped): A PoseStamped object representing the goal pose.
    """

    def __init__(self):
        """Initialize the GoToGoal class."""
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints')

        # Initialize a list to store waypoints
        self.waypoints = []

        # Create and append waypoints to the list
        self.create_waypoints()

    def create_waypoints(self):
        """Create a sequence of waypoints."""
        for x in range(0, 3):
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = "map"
            goal.pose.position.x = 0.5 * x
            goal.pose.position.y = 0.0
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0
            self.waypoints.append(goal)

    def send_goal(self):
        """Send the goal to the action server."""
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        # Wait for the action server to become available
        self._action_client.wait_for_server()

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # Register a callback to handle the response from the action server
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal Accepted')

        # Wait for the result from the action server
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result from the action server."""
        result = future.result().result
        self.get_logger().info(
            f"Result -> Missed Waypoints: {len(result.missed_waypoints)}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Handle the feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback -> Current Waypoint: {feedback.current_waypoint}", throttle_duration_sec=2.0)


def main(args=None):
    """Main function to initialize and run the GoToGoal node."""
    rclpy.init(args=args)

    # Create an instance of the GoToGoal class
    gotogoal = GoToGoal()

    # Send the goal to the action server
    gotogoal.send_goal()

    # Spin the node to process callbacks
    rclpy.spin(gotogoal)


if __name__ == '__main__':
    main()
