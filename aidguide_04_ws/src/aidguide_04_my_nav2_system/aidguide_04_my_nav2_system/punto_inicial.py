import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class Publisher(Node):
    """
    Nodo que publica una posición inicial en el topic `/initialpose` para establecer
    la localización inicial del robot en el mapa.
    """

    def __init__(self):
        """
        Inicializa el nodo, el publicador y un temporizador para ejecutar la publicación periódica.
        """

        super().__init__('initial_pose_pub_node')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        timer_period = 0.5  # seconds
        self.timer_ = self.create_timer(timer_period, self.callback)

    def callback(self):
        """
        Crea y publica un mensaje `PoseWithCovarianceStamped` con la posición inicial del robot.
        """

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # ✅ Muy importante
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x =  -3.414589 
        msg.pose.pose.position.y = -1.716389
        msg.pose.pose.orientation.w = 1.0
        self.get_logger().info('📍 Publicando posición inicial: X=-2.284589, Y=-3.375697, W=1.0')
        self.publisher_.publish(msg)

def main(args=None):
    """
    Función principal que inicializa el nodo y ejecuta una única iteración de publicación.

    Args:
        args: Argumentos de línea de comandos (opcional).
    """

    rclpy.init(args=args)
    publisher = Publisher()
    try:
        rclpy.spin_once(publisher, timeout_sec=2.0)  # ✅ Evita que se repita infinitamente
    except KeyboardInterrupt:
        publisher.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
