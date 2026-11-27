
import rclpy
import math

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ==================================================
# WAYPOINTS : (x, y, qz, qw)
# Repère : map
# ==================================================

WAYPOINTS = [
    # Point A (yaw ≈ -151.93°)
    (-1.254, -0.925, 0.970, -0.243),

    # Point B (yaw ≈ 75.14°)
    (-0.300, 0.242, 0.610, 0.793),

    # Point C (yaw ≈ 167.28°)
    (-1.461, 1.109, 0.994, 0.111),
]


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.waypoint_index = 0
        self.goal_handle = None

        self.start_timer = self.create_timer(
            1.0,
            self.start_navigation
        )

        self.get_logger().info(
            'Waypoint navigator initialized.'
        )

    # ==================================================
    # ATTENTE DE NAV2
    # ==================================================

    def start_navigation(self):

        if not self.nav_client.server_is_ready():
            self.get_logger().info(
                'Waiting for Nav2...'
            )
            return

        self.start_timer.cancel()

        self.get_logger().info(
            'Nav2 is ready!'
        )

        self.send_next_waypoint()

    # ==================================================
    # ENVOI DU WAYPOINT
    # ==================================================

    def send_next_waypoint(self):

        if self.waypoint_index >= len(WAYPOINTS):

            self.get_logger().info(
                'All waypoints reached!'
            )
            return

        x, y, qz, qw = WAYPOINTS[self.waypoint_index]

        self.get_logger().info(
            f'Sending waypoint {self.waypoint_index + 1}'
        )

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose = PoseStamped()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        future = self.nav_client.send_goal_async(
            goal_msg
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # ==================================================
    # REPONSE DE NAV2
    # ==================================================

    def goal_response_callback(self, future):

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:

            self.get_logger().error(
                f'Waypoint {self.waypoint_index + 1} rejected!'
            )
            return

        self.get_logger().info(
            f'Waypoint {self.waypoint_index + 1} accepted.'
        )

        result_future = self.goal_handle.get_result_async()

        result_future.add_done_callback(
            self.navigation_result_callback
        )

    # ==================================================
    # RESULTAT DE NAVIGATION
    # ==================================================

    def navigation_result_callback(self, future):

        result = future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:

            self.get_logger().info(
                f'Waypoint {self.waypoint_index + 1} reached!'
            )

            self.waypoint_index += 1

            self.send_next_waypoint()

        else:

            self.get_logger().error(
                f'Navigation failed at waypoint '
                f'{self.waypoint_index + 1}. '
                f'Status: {result.status}'
            )

            self.get_logger().error(
                'Stopping waypoint sequence.'
            )


def main(args=None):

    rclpy.init(args=args)

    navigator = WaypointNavigator()

    try:
        rclpy.spin(navigator)

    except KeyboardInterrupt:
        pass

    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
