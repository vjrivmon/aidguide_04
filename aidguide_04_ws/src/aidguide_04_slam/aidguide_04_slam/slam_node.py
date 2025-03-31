#!/usr/bin/env python3
"""
Nodo para gestionar el SLAM (Simultaneous Localization and Mapping) en AidGuide 04.
Este nodo proporciona una interfaz para interactuar con slam_toolbox.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
from ament_index_python.packages import get_package_share_directory

class SlamNode(Node):
    """
    Clase que implementa el nodo de SLAM para AidGuide 04.
    Provee funcionalidades para controlar el proceso de SLAM.
    """
    def __init__(self):
        super().__init__('aidguide_slam_node')
        self.get_logger().info('Iniciando nodo de SLAM...')
        
        # Suscriptores
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/slam_toolbox/pose',
            self.pose_callback,
            10)
        
        # Estado
        self.map_received = False
        
        # Timer para reportar estado
        self.timer = self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('Nodo de SLAM inicializado correctamente')
        
    def map_callback(self, msg):
        """
        Callback para procesar mensajes del mapa.
        
        Args:
            msg: Mensaje OccupancyGrid con el mapa actual
        """
        if not self.map_received:
            self.map_received = True
            self.get_logger().info('Primer mapa recibido')
        
    def pose_callback(self, msg):
        """
        Callback para procesar mensajes de pose.
        
        Args:
            msg: Mensaje PoseWithCovarianceStamped con la pose actual
        """
        # Solo para propósitos de depuración, no imprimir en producción
        # para evitar saturar los logs
        pass
        
    def status_callback(self):
        """
        Reporta el estado del nodo periódicamente.
        """
        if self.map_received:
            self.get_logger().info('Estado SLAM: Mapa disponible')
        else:
            self.get_logger().info('Estado SLAM: Esperando mapa...')

def main(args=None):
    """
    Función principal del nodo.
    """
    rclpy.init(args=args)
    slam_node = SlamNode()
    
    try:
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        slam_node.get_logger().info('Interrumpido por el usuario')
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 