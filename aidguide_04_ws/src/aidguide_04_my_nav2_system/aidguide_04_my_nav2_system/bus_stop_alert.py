import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

class BusStopAlertNode(Node):
    """
    Nodo que detecta paradas de autobús cercanas y permite al usuario modificar la ruta.
    """

    def __init__(self):
        """
        Inicializa el nodo, el cliente de acción y la suscripción a odometría.
        """
        super().__init__('bus_stop_alert_node')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.current_position = None
        self.bus_stops = self.define_bus_stops()
        self.proximity_threshold = 0.5  # Distancia en metros para detectar paradas
        self.current_waypoints = self.define_default_waypoints()
        self.get_logger().info('Bus Stop Alert Node initialized')

    def define_bus_stops(self):
        """
        Define las coordenadas de las paradas de autobús.

        Returns:
            list: Lista de diccionarios con coordenadas de paradas.
        """
        return [
            {'name': 'Bus Stop 1', 'x': 1.5, 'y': -1.4},
            {'name': 'Bus Stop 2', 'x': 2.0, 'y': -1.5}
        ]

    def define_default_waypoints(self):
        """
        Define los waypoints por defecto (basados en el script original).

        Returns:
            list: Lista de objetos PoseStamped con los waypoints.
        """
        waypoints = [
            self.create_pose(1.261, -2.611, 0.0, 1.0),
            self.create_pose(1.26, -2.60, 0.0, 1.0),
            self.create_pose(1.270897, -1.390872, 0.0, 1.0),
            self.create_pose(1.508040, -1.390872, 0.0, 1.0),
            self.create_pose(1.928040, -1.398490, 0.0, 1.0),
            self.create_pose(1.967944, 3.029384, 0.0, 1.0)
        ]
        return waypoints

    def create_pose(self, x, y, z, w):
        """
        Crea un objeto PoseStamped con las coordenadas dadas.

        Args:
            x, y, z (float): Coordenadas de posición.
            w (float): Orientación (cuaternión w).

        Returns:
            PoseStamped: Objeto PoseStamped configurado.
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = w
        return pose

    def odom_callback(self, msg):
        """
        Actualiza la posición actual del robot y verifica la proximidad a paradas.

        Args:
            msg (Odometry): Mensaje de odometría recibido.
        """
        self.current_position = msg.pose.pose.position
        self.check_bus_stop_proximity()

    def check_bus_stop_proximity(self):
        """
        Verifica si el robot está cerca de una parada de autobús y emite una alerta.
        """
        if self.current_position is None:
            return

        for stop in self.bus_stops:
            distance = math.sqrt(
                (self.current_position.x - stop['x'])**2 +
                (self.current_position.y - stop['y'])**2
            )
            if distance < self.proximity_threshold:
                self.get_logger().info(f'Cerca de {stop["name"]} a {distance:.2f}m')
                self.prompt_user_for_bus_stop(stop)

    def prompt_user_for_bus_stop(self, stop):
        """
        Pregunta al usuario si desea agregar la parada de autobús como waypoint.

        Args:
            stop (dict): Información de la parada de autobús.
        """
        try:
            user_response = input(f"¿Desea ir a {stop['name']}? (sí/no): ").lower()
            if user_response == 'sí':
                self.get_logger().info(f'Agregando {stop["name"]} a la ruta.')
                self.modify_route(stop['x'], stop['y'])
        except KeyboardInterrupt:
            self.get_logger().info('Interacción cancelada por el usuario.')

    def modify_route(self, x, y):
        """
        Modifica la ruta agregando un nuevo waypoint para la parada de autobús.

        Args:
            x, y (float): Coordenadas de la parada de autobús.
        """
        # Cancelar la acción actual si existe
        if hasattr(self, '_goal_handle') and self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

        # Crear un nuevo waypoint para la parada de autobús
        bus_stop_pose = self.create_pose(x, y, 0.0, 1.0)
        # Mantener los waypoints restantes
        new_waypoints = [bus_stop_pose] + self.current_waypoints
        self.current_waypoints = new_waypoints
        self.send_waypoints(new_waypoints)

    def send_waypoints(self, waypoints):
        """
        Envía la lista de waypoints al action server.

        Args:
            waypoints (list): Lista de objetos PoseStamped a enviar.
        """
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server no disponible')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info('Enviando waypoints al servidor...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Maneja la respuesta inicial del action server.

        Args:
            future: Objeto Future con la respuesta del servidor.
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted by server, waiting for result...')
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Maneja el feedback enviado por el servidor.

        Args:
            feedback_msg: Mensaje de feedback recibido.
        """
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Currently at waypoint: {current_waypoint}')

    def get_result_callback(self, future):
        """
        Procesa el resultado final de la acción.

        Args:
            future: Objeto Future con el resultado.
        """
        result = future.result().result
        self.get_logger().info('Result received:')
        self.get_logger().info(f'Missed waypoints: {len(result.missed_waypoints)}')
        self.get_logger().info(f'Error code: {result.error_code}')

def main(args=None):
    """
    Función principal para inicializar y ejecutar el nodo.
    """
    rclpy.init(args=args)
    bus_stop_node = BusStopAlertNode()
    bus_stop_node.send_waypoints(bus_stop_node.current_waypoints)  # Start navigation
    try:
        rclpy.spin(bus_stop_node)
    except KeyboardInterrupt:
        bus_stop_node.get_logger().info('Nodo detenido por el usuario.')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()