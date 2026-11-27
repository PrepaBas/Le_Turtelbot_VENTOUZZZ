import rclpy
import math
import random

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan


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


# ==================================================
# PARAMÈTRES DE LA CALIBRATION (errance aléatoire)
# ==================================================

CALIB_DURATION = 90.0     # s : durée totale de l'errance
LINEAR_SPEED = 0.15       # m/s : vitesse d'avance
TURN_SPEED = 0.6          # rad/s : vitesse de rotation devant un mur

STOP_DIST = 0.25          # m : mur devant en dessous de cette distance
SIDE_BLOCKED_DIST = 0.50  # m : côté considéré bloqué en dessous

FRONT_HALF_ANGLE = math.radians(30)   # secteur avant : ±30°
SIDE_MIN_ANGLE = math.radians(30)     # secteurs latéraux : de 30° à 90°
SIDE_MAX_ANGLE = math.radians(90)


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # Une seule souscription, QoS capteur (BEST_EFFORT)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.waypoint_index = 0
        self.goal_handle = None

        # --- État de la calibration ---
        self.calibration_done = False
        # Publication des vitesses pendant la calibration
        # (Jazzy : /cmd_vel est de type TwistStamped)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            'cmd_vel',
            10
        )

        self.calib_start_time = None   # s, fixé au premier scan
        self.avoid_dir = 0.0           # sens de pivot verrouillé (+1 gauche / -1 droite / 0 aucun)

        # Le timer tourne dès le départ, mais start_navigation
        # attend que la calibration soit terminée
        self.start_timer = self.create_timer(
            1.0,
            self.start_navigation
        )

        self.get_logger().info(
            'Waypoint navigator initialized.'
        )

    # ==================================================
    # SCAN (appelé à chaque message sur /scan)
    # ==================================================

    def scan_callback(self, msg):

        if self.calibration_done:
            # TODO plus tard : détection de quasi-collision
            return

        if self.process_calibration_scan(msg):
            self.calibration_done = True
            self.get_logger().info(
                'Calibration finished.'
            )

    # ==================================================
    # SÉQUENCE DE CALIBRATION
    # ==================================================

    def process_calibration_scan(self, msg):
        """
        Traite un scan pendant la calibration.
        Retourne True quand la calibration est terminée,
        False sinon.
        """
        now = self.get_clock().now().nanoseconds / 1e9

        if self.calib_start_time is None:
            self.calib_start_time = now
            self.get_logger().info('Starting calibration wander...')

        elapsed = now - self.calib_start_time

        # --- Fin de la calibration : on arrête le robot ---
        if elapsed >= CALIB_DURATION:
            self.publish_velocity(0.0, 0.0)
            return True

        # --- Distances minimales par secteur (mesures valides seulement) ---
        front = self.sector_min(msg, -FRONT_HALF_ANGLE, FRONT_HALF_ANGLE)
        left = self.sector_min(msg, SIDE_MIN_ANGLE, SIDE_MAX_ANGLE)
        right = self.sector_min(msg, -SIDE_MAX_ANGLE, -SIDE_MIN_ANGLE)

        if front < STOP_DIST:
            # Mur devant : on choisit un sens de pivot une seule fois,
            # puis on le garde jusqu'à ce que l'avant soit dégagé.
            if self.avoid_dir == 0.0:
                left_blocked = left < SIDE_BLOCKED_DIST
                right_blocked = right < SIDE_BLOCKED_DIST

                if right_blocked and not left_blocked:
                    self.avoid_dir = 1.0    # bloqué à droite -> gauche
                elif left_blocked and not right_blocked:
                    self.avoid_dir = -1.0   # bloqué à gauche -> droite
                else:
                    self.avoid_dir = random.choice((1.0, -1.0))

            self.publish_velocity(0.0, self.avoid_dir * TURN_SPEED)
        else:
            # Voie libre : tout droit
            self.avoid_dir = 0.0
            self.publish_velocity(LINEAR_SPEED, 0.0)

        return False

    def publish_velocity(self, linear, angular):
        """Publie une commande de vitesse (TwistStamped, Jazzy)."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular
        self.cmd_vel_pub.publish(cmd)

    def sector_min(self, msg, angle_a, angle_b):
        """
        Distance valide minimale entre angle_a et angle_b (radians,
        0 = avant du robot, positif = vers la gauche).
        Retourne range_max si aucune mesure valide dans le secteur.
        """
        best = msg.range_max

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))  # [-pi, pi]

            if angle_a <= angle <= angle_b and r < best:
                best = r

        return best

    # ==================================================
    # ATTENTE DE LA CALIBRATION PUIS DE NAV2
    # ==================================================

    def start_navigation(self):

        if not self.calibration_done:
            self.get_logger().info(
                'Calibration in progress...'
            )
            return

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
