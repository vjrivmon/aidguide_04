

"""Módulo para la clase AidguideNavigation.



Este módulo proporciona funcionalidades para el proyecto AidGuide 04.

"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AidguideNavigation(Node):
    '"""Clase AidguideNavigation.\n\nImplementa funcionalidad para AidguideNavigation.\n"""'

    def __init__(self):
        '"""  init  .\n\n"""'
        super().__init__('aidguide_navigation')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        '"""Timer callback.\n\n"""'
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info(('Publishing: "%s"' % msg))

def main(args=None):
    '"""Main.\n\nArgs:\n    args (Any): Descripción del parámetro.\n"""'
    rclpy.init(args=args)
    publisher = AidguideNavigation()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
if (__name__ == '__main__'):
    main()
