import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
import threading
import queue
import time

class BusStopAlertNode(Node):
    """
    Nodo simple que detecta paradas, pregunta si ir, y termina en la parada si se elige si.
    """
    def __init__(self):
        super().__init__('bus_stop_alert_node')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.decision_sub = self.create_subscription(
            String, '/user_decision', self.decision_callback, 10)
        self.current_position = None
        self.last_position = None
        self.bus_stops = [
            {'name': 'Parada 1', 'x': 1.5, 'y': -1.4},
            {'name': 'Parada 2', 'x': 2.0, 'y': -1.5}
        ]
        self.proximity_threshold = 2.0
        self.cooldown_period = 120.0
        self.last_detection = 0.0
        self.current_waypoints = [
            self._create_pose(1.26, -2.61, 0.0, 1.0),
            self._create_pose(1.27, -1.39, 0.0, 1.0),
            self._create_pose(1.97, 3.03, 0.0, 1.0)
        ]
        self.goal_handle = None
        self.is_waiting = False
        self.is_done = False
        self.nearest_stop = None
        self.input_queue = queue.Queue()
        self.decision_timeout = 60.0
        self.decision_start = None
        self.get_logger().info('Nodo inicializado. Use /robot1/follow_waypoints si hay errores.')
        self.create_timer(2.0, self.start_navigation)

    def _create_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = w
        return pose

    def start_navigation(self):
        if not self.is_waiting and not self.is_done and self.goal_handle is None:
            if not self.action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Servidor no disponible.')
                return
            self._send_waypoints(self.current_waypoints)

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        if not self.is_waiting and not self.is_done:
            self._check_proximity()

    def _check_proximity(self):
        if self.current_position is None:
            return
        current_time = time.time()
        if current_time - self.last_detection < self.cooldown_period:
            if self.last_position and self.current_position:
                dist_moved = math.sqrt(
                    (self.current_position.x - self.last_position.x)**2 +
                    (self.current_position.y - self.last_position.y)**2
                )
                if dist_moved < 1.5:
                    return
        for stop in self.bus_stops:
            distance = math.sqrt(
                (self.current_position.x - stop['x'])**2 +
                (self.current_position.y - stop['y'])**2
            )
            if distance < self.proximity_threshold:
                self.get_logger().info(f'Cerca de {stop["name"]} a {distance:.2f}m')
                self.is_waiting = True
                self.nearest_stop = stop
                self.decision_start = current_time
                self.last_detection = current_time
                self.last_position = self.current_position
                if self.goal_handle is not None:
                    self.get_logger().info('Cancelando navegacion...')
                    cancel_future = self.goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(self._cancel_done)
                else:
                    self._prompt_user()
                break

    def _cancel_done(self, future):
        self.get_logger().info('Navegacion cancelada.')
        self.goal_handle = None
        self._prompt_user()

    def _prompt_user(self):
        if self.nearest_stop:
            self.get_logger().info(f'Esperando 60s: Desea ir a {self.nearest_stop["name"]}? (si/no)')
            threading.Thread(target=self._get_input, daemon=True).start()

    def _get_input(self):
        try:
            response = input(f'Desea ir a {self.nearest_stop["name"]}? (si/no): ').strip().lower()
            if response == 'si':
                response = 'sí'
            self.input_queue.put(response)
        except Exception:
            self.get_logger().info('Error en terminal. Publique en /user_decision.')
            self.input_queue.put('no')

    def decision_callback(self, msg):
        if self.is_waiting and self.nearest_stop:
            response = msg.data.strip().lower()
            if response == 'si':
                response = 'sí'
            self.input_queue.put(response)

    def _send_waypoints(self, waypoints):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        self.get_logger().info('Enviando waypoints...')
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.get_logger().warn('Goal rechazado.')
                self.goal_handle = None
            else:
                self.get_logger().info('Goal aceptado.')
                result_future = self.goal_handle.get_result_async()
                result_future.add_done_callback(self._result_callback)
        except Exception:
            self.get_logger().error('Error en goal.')
            self.goal_handle = None

    def _result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'Navegacion completa. Waypoints omitidos: {len(result.missed_waypoints)}')
            self.goal_handle = None
            self.is_done = True
        except Exception:
            self.get_logger().error('Error en resultado.')
            self.goal_handle = None
            self.is_done = True

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.is_waiting and self.nearest_stop:
                if time.time() - self.decision_start > self.decision_timeout:
                    self.get_logger().info('Tiempo agotado. Continuando...')
                    self.input_queue.put('no')
                try:
                    response = self.input_queue.get_nowait()
                    if response not in ['sí', 'no']:
                        self.get_logger().warn('Use "si" o "no".')
                        self.decision_start = time.time()
                        self._prompt_user()
                        continue
                    self.is_waiting = False
                    if response == 'sí':
                        self.get_logger().info(f'Llevando a {self.nearest_stop["name"]}...')
                        stop_pose = self._create_pose(self.nearest_stop['x'], self.nearest_stop['y'], 0.0, 1.0)
                        self._send_waypoints([stop_pose])
                    else:
                        self.get_logger().info('Continuando ruta original...')
                        self._send_waypoints(self.current_waypoints)
                    self.nearest_stop = None
                    self.decision_start = None
                except queue.Empty:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = BusStopAlertNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Nodo detenido.')
    finally:
        if node.goal_handle:
            node.goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()