import rclpy
from rclpy.node import Node            # importamos las librerias ROS2 de python
from geometry_msgs.msg import Twist    # importamos los mensajes tipo Twist


# creamos una clase pasándole como parámetro el Nodo
class AidguideNavigation(Node):

    def __init__(self):
        # Constructor de la clase
        # ejecutamos super() para inicializar el Nodo
        # introducimos le nombre del nodo como parámetro
        super().__init__('aidguide_navigation')
        # creamos el objeto publisher
        # que publicara en el topic /cmd_vel
        # la cola del topic es de 10 mensajes
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # definimos un periodo para publicar periodicamente
        timer_period = 0.5
        # creamos un timer con dos parametros:
        # - el periodo (0.5 seconds)
        # - la funcion a realizar  (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()                   # creamos el mensaje tipo Twist
        msg.linear.x = 0.5              # define la velocidad lineal en el eje x
        msg.angular.z = 0.5             # define tla velocidad angular en el eje z
        self.publisher_.publish(msg)    # Publicamos el mensaje en el topic
        self.get_logger().info('Publishing: "%s"' % msg)        # Mostramos el mensaje por el terminal


def main(args=None):
    rclpy.init(args=args)               # inicializa la comunicación
    publisher = AidguideNavigation()    # declara el constructor del nodo
    rclpy.spin(publisher)               # dejamos vivo el nodo, para parar el programa habrá que matar el node (ctrl+c)
    publisher.destroy_node()            # destruye en nodo
    rclpy.shutdown()                    # se cierra la comunicacion ROS


if __name__ == '__main__':
    main()