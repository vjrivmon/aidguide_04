#!/usr/bin/env python3
"""
Nodo para gestionar la localización en AidGuide 04.
Este nodo proporciona una interfaz para interactuar con el sistema de localización (AMCL).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_srvs.srv import Empty
import os

class LocalizationNode(Node):
    """
    Clase que implementa el nodo de localización para AidGuide 04.
    Provee funcionalidades para controlar el proceso de localización.
    """
    def __init__(self):
        super().__init__('aidguide_localization_node')
        self.get_logger().info('Iniciando nodo de localización...')
        
        # Suscriptores
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        # Publicadores
        self.status_pub = self.create_publisher(
            String, 
            '/localization/status', 
            10)
        
        # Clientes de servicios
        self.global_loc_client = self.create_client(
            Empty, 
            '/reinitialize_global_localization')
        
        # Estado
        self.localized = False
        self.map_available = False
        
        # Timer para reportar estado
        self.timer = self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info('Nodo de localización inicializado correctamente')
    
    def request_global_localization(self):
        """
        Solicita una relocalización global para determinar la posición del robot.
        """
        if not self.global_loc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Servicio de relocalización global no disponible')
            return False
            
        request = Empty.Request()
        future = self.global_loc_client.call_async(request)
        self.get_logger().info('Solicitada relocalización global')
        return True
        
    def pose_callback(self, msg):
        """
        Callback para procesar mensajes de pose.
        
        Args:
            msg: Mensaje PoseWithCovarianceStamped con la pose estimada
        """
        # Análisis básico de la covarianza para determinar si estamos bien localizados
        # Una covarianza baja indica mayor certeza en la localización
        cov = msg.pose.covariance
        
        # Un enfoque simple: comprobar los elementos diagonales principales
        # para posición x, y y orientación yaw
        pos_uncertainty = cov[0] + cov[7]  # Covarianza en X e Y
        orient_uncertainty = cov[35]       # Covarianza en Yaw
        
        # Valores umbral (estos deberían ajustarse según la aplicación)
        if pos_uncertainty < 0.5 and orient_uncertainty < 0.2:
            if not self.localized:
                self.localized = True
                self.get_logger().info('Robot localizado con alta confianza')
        else:
            if self.localized:
                self.localized = False
                self.get_logger().info('Confianza en la localización reducida')
    
    def map_callback(self, msg):
        """
        Callback para procesar mensajes del mapa.
        
        Args:
            msg: Mensaje OccupancyGrid con el mapa actual
        """
        if not self.map_available:
            self.map_available = True
            map_size = len(msg.data)
            self.get_logger().info(f'Mapa disponible, tamaño: {map_size} celdas')
    
    def status_callback(self):
        """
        Reporta el estado del nodo periódicamente.
        """
        status_msg = String()
        
        if not self.map_available:
            status_msg.data = "ESPERANDO_MAPA"
        elif not self.localized:
            status_msg.data = "LOCALIZANDO"
        else:
            status_msg.data = "LOCALIZADO"
            
        self.status_pub.publish(status_msg)
        self.get_logger().debug(f'Estado de localización: {status_msg.data}')

def main(args=None):
    """
    Función principal del nodo.
    """
    rclpy.init(args=args)
    localization_node = LocalizationNode()
    
    try:
        rclpy.spin(localization_node)
    except KeyboardInterrupt:
        localization_node.get_logger().info('Interrumpido por el usuario')
    finally:
        localization_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 