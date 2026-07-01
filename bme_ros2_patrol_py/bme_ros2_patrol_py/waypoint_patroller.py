import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class WaypointPatroller(Node):
    def __init__(self):
        super().__init__('waypoint_patroller')
        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        self.waypoints_xytheta = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 1.57),
            (0.0, 1.0, 3.14),
        ]

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def send_waypoints(self):
        self.get_logger().info('Waiting for FollowWaypoints action server...')
        self.client.wait_for_server()

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = [
            self.make_pose(x, y, yaw)
            for x, y, yaw in self.waypoints_xytheta
        ]

        for i in range(len(goal_msg.poses)):
            self.get_logger().info(f'Navigating to Waypoint {i + 1}...')

        future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint

        if not hasattr(self, 'last_waypoint') or self.last_waypoint != current:
            self.last_waypoint = current
            self.get_logger().info(f'Traveling toward Waypoint {current + 1}...')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Waypoint goal rejected.')
            return

        self.get_logger().info('Waypoint goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        missed = list(result.missed_waypoints)

        if missed:
            self.get_logger().warn(f'Missed waypoints: {missed}')
        else:
            for i in range(len(self.waypoints_xytheta)):
                self.get_logger().info(f'Waypoint {i + 1} Reached!')
            self.get_logger().info('Patrol complete!')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPatroller()
    node.send_waypoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
