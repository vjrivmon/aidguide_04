import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

class PuntoAPuntoNode(Node):
    def __init__(self):
        super().__init__('punto_a_punto_node')
        self._client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Esperamos a que el servidor de acciones esté disponible
        self._client.wait_for_server()
        self.get_logger().info("Servidor NavigateToPose disponible. Enviando goal...")

        # Ejemplo: Enviamos un objetivo (x=2.0, y=0.0, orientado con w=1.0)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Frame en el que se define la pose (generalmente "map")
        goal_msg.pose.header.frame_id = 'map'
        
        # Marca de tiempo
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('La meta ha sido rechazada :(')
            return

        self.get_logger().info('La meta ha sido aceptada :)')
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'El resultado de la navegación es: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Aquí puedes imprimir info sobre el feedback
        self.get_logger().info(f'Feedback de la navegación: distancia restante = {feedback.distance_remaining}')

def main(args=None):
    rclpy.init(args=args)
    node = PuntoAPuntoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
