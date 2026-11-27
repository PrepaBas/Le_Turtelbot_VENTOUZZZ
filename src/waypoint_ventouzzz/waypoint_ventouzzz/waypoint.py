import rclpy
import math
import random

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, Twist
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

CALIB_DURATION = 30.0     # s : durée totale de l'errance
LINEAR_SPEED = 0.15       # m/s : vitesse d'avance normale
SLOW_SPEED = 0.07         # m/s : vitesse quand un obstacle approche
TURN_SPEED = 0.6          # rad/s : vitesse de rotation d'évitement
WANDER_TURN_MAX = 0.35    # rad/s : amplitude de la dérive aléatoire
WANDER_PERIOD = (2.0, 4.0)  # s : intervalle entre deux changements de dérive

STOP_DIST = 0.35          # m : en dessous, on s'arrête d'avancer et on pivote
SLOW_DIST = 0.70          # m : en dessous, on ralentit

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
        # (si ton Nav2 attend du TwistStamped, remplace Twist par TwistStamped)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.calib_start_time = None   # s, fixé au premier scan
        self.wander_turn = 0.0         # dérive angulaire courante
        self.next_wander_change = 0.0  # s écoulées avant le prochain tirage
        self.avoid_dir = 0.0           # sens de pivot verrouillé (+1 / -1 / 0)

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
            self.cmd_vel_pub.publish(Twist())
            return True

        # --- Distances minimales par secteur (mesures valides seulement) ---
        front = self.sector_min(msg, -FRONT_HALF_ANGLE, FRONT_HALF_ANGLE)
        left = self.sector_min(msg, SIDE_MIN_ANGLE, SIDE_MAX_ANGLE)
        right = self.sector_min(msg, -SIDE_MAX_ANGLE, -SIDE_MIN_ANGLE)

        cmd = Twist()

        if front < STOP_DIST:
            # Obstacle proche : pivot sur place vers le côté le plus dégagé.
            # Le sens est verrouillé tant qu'on n'est pas dégagé pour éviter
            # d'osciller gauche/droite.
            if self.avoid_dir == 0.0:
                self.avoid_dir = 1.0 if left >= right else -1.0
            cmd.linear.x = 0.0
            cmd.angular.z = self.avoid_dir * TURN_SPEED
        else:
            self.avoid_dir = 0.0

            # Dérive aléatoire : on retire une nouvelle valeur régulièrement
            if elapsed >= self.next_wander_change:
                self.wander_turn = random.uniform(
                    -WANDER_TURN_MAX, WANDER_TURN_MAX
                )
                self.next_wander_change = elapsed + random.uniform(
                    *WANDER_PERIOD
                )

            cmd.linear.x = SLOW_SPEED if front < SLOW_DIST else LINEAR_SPEED
            cmd.angular.z = self.wander_turn

            # Léger biais pour s'écarter d'un mur latéral trop proche
            if left < SLOW_DIST and left < right:
                cmd.angular.z -= 0.2
            elif right < SLOW_DIST and right < left:
                cmd.angular.z += 0.2

        self.cmd_vel_pub.publish(cmd)
        return False

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
