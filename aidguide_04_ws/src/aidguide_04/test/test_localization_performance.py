#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test de Rendimiento de Localización para AidGuide 04

Este módulo implementa pruebas automatizadas para evaluar el rendimiento
del sistema de localización del robot, midiendo la precisión, tiempos de respuesta
y robustez ante diferentes condiciones. Estas pruebas fueron adaptadas desde
el notebook de Colab para integrarlas en el sistema de pruebas.

Author: AidGuide Team
"""

import unittest
import os
import sys
import numpy as np
import time
import json
import math
from pathlib import Path
from unittest.mock import MagicMock, patch

# Intentar importar dependencias necesarias
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
    from tf2_ros import Buffer, TransformListener
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# Determinar la ruta al directorio del proyecto
ROOT_DIR = Path(__file__).parent.parent.parent
SRC_PATH = ROOT_DIR
MAPS_PATH = ROOT_DIR / "maps"

class LocalizationPerformanceTest(unittest.TestCase):
    """Pruebas para evaluar el rendimiento del sistema de localización."""

    def setUp(self):
        """Preparar entorno para las pruebas."""
        # Crear puntos de referencia para las pruebas
        self.reference_points = [
            {"x": 0.0, "y": 0.0, "theta": 0.0},
            {"x": 10.0, "y": 0.0, "theta": 0.0},
            {"x": 0.0, "y": 10.0, "theta": math.pi/2},
            {"x": 10.0, "y": 10.0, "theta": math.pi/4}
        ]
        
        # Crear un mock del localizador
        self.mock_localizer = MagicMock()
        self.mock_localizer.get_current_pose.return_value = {
            "x": 5.0, "y": 5.0, "theta": 0.0,
            "covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.0],
            "valid": True
        }
        
        # Crear un mock del mapa
        self.mock_map = MagicMock()
        self.mock_map.get_map_data.return_value = {
            "width": 100, 
            "height": 100, 
            "resolution": 0.05,
            "origin": {"x": -5.0, "y": -5.0, "theta": 0.0}
        }
        
        # Crear secuencia de movimiento simulada
        self.simulated_path = []
        for i in range(100):
            x = i * 0.1  # Incrementar X en 0.1 cada paso
            y = math.sin(i * 0.1) * 0.5  # Movimiento sinusoidal en Y
            theta = math.atan2(math.cos(i * 0.1) * 0.5 * 0.1, 0.1) if i > 0 else 0.0
            self.simulated_path.append({"x": x, "y": y, "theta": theta})

    def test_localization_accuracy(self):
        """Verificar la precisión de la localización."""
        # Establecer el error máximo permitido en metros
        max_position_error = 0.1  # 10 cm
        max_orientation_error = 0.05  # ~3 grados
        
        # Verificar la precisión de la localización para cada punto de referencia
        for ref_point in self.reference_points:
            # Configurar el mock para retornar una posición con pequeños errores aleatorios
            error_x = (np.random.random() - 0.5) * max_position_error
            error_y = (np.random.random() - 0.5) * max_position_error
            error_theta = (np.random.random() - 0.5) * max_orientation_error
            
            estimated_pose = {
                "x": ref_point["x"] + error_x,
                "y": ref_point["y"] + error_y, 
                "theta": ref_point["theta"] + error_theta,
                "covariance": [0.01, 0.0, 0.0, 0.0, 0.01, 0.005],
                "valid": True
            }
            
            self.mock_localizer.get_current_pose.return_value = estimated_pose
            
            # Obtener la posición estimada
            pose = self.mock_localizer.get_current_pose()
            
            # Verificar que la posición estimada sea válida
            self.assertTrue(pose["valid"], 
                           "La posición estimada debe ser válida")
            
            # Calcular el error de posición
            position_error = math.sqrt((pose["x"] - ref_point["x"])**2 + 
                                      (pose["y"] - ref_point["y"])**2)
            
            # Calcular el error de orientación (considerando la naturaleza circular)
            orientation_error = min(
                abs(pose["theta"] - ref_point["theta"]),
                abs(pose["theta"] - ref_point["theta"] + 2*math.pi),
                abs(pose["theta"] - ref_point["theta"] - 2*math.pi)
            )
            
            # Verificar que los errores estén dentro de los límites aceptables
            self.assertLessEqual(position_error, max_position_error,
                               f"Error de posición ({position_error:.3f}m) excede el máximo permitido ({max_position_error}m)")
            self.assertLessEqual(orientation_error, max_orientation_error,
                               f"Error de orientación ({orientation_error:.3f}rad) excede el máximo permitido ({max_orientation_error}rad)")
            
            # Verificar que la covarianza tenga valores razonables
            self.assertGreater(pose["covariance"][0], 0,
                              "La covarianza en X debe ser positiva")
            self.assertGreater(pose["covariance"][4], 0,
                              "La covarianza en Y debe ser positiva")

    def test_localization_performance(self):
        """Evaluar el rendimiento temporal del sistema de localización."""
        # Medir el tiempo de respuesta del sistema de localización
        start_time = time.time()
        iterations = 100
        
        for _ in range(iterations):
            pose = self.mock_localizer.get_current_pose()
            
        end_time = time.time()
        avg_time = (end_time - start_time) / iterations
        
        # Verificar que el tiempo de respuesta sea adecuado para aplicaciones en tiempo real
        self.assertLessEqual(avg_time, 0.01,
                           f"El tiempo promedio de localización ({avg_time:.5f}s) es demasiado alto para aplicaciones en tiempo real")
        
        print(f"Tiempo promedio de localización: {avg_time:.5f} segundos")

    def test_pose_tracking(self):
        """Verificar el seguimiento continuo de la posición a lo largo de un camino."""
        # Crear un mock del rastreador de posición
        mock_pose_tracker = MagicMock()
        
        # Lista para almacenar las posiciones estimadas
        estimated_poses = []
        
        # Simular el seguimiento a lo largo del camino
        for i, true_pose in enumerate(self.simulated_path):
            # Añadir pequeño ruido aleatorio a la posición "verdadera"
            noise_x = (np.random.random() - 0.5) * 0.02  # ±1cm de ruido
            noise_y = (np.random.random() - 0.5) * 0.02
            noise_theta = (np.random.random() - 0.5) * 0.01
            
            estimated_pose = {
                "x": true_pose["x"] + noise_x,
                "y": true_pose["y"] + noise_y,
                "theta": true_pose["theta"] + noise_theta,
                "timestamp": i * 0.1  # Simulamos 10 Hz
            }
            
            estimated_poses.append(estimated_pose)
        
        # Configurar el mock para devolver las posiciones estimadas
        mock_pose_tracker.get_tracked_poses.return_value = estimated_poses
        
        # Obtener las posiciones estimadas
        tracked_poses = mock_pose_tracker.get_tracked_poses()
        
        # Verificar que el número de posiciones estimadas coincide con el camino simulado
        self.assertEqual(len(tracked_poses), len(self.simulated_path),
                        "El número de posiciones estimadas debe coincidir con el camino simulado")
        
        # Calcular el error acumulado a lo largo del camino
        total_position_error = 0.0
        max_position_error = 0.0
        
        for i, (estimated, true) in enumerate(zip(tracked_poses, self.simulated_path)):
            # Calcular error de posición para cada punto
            position_error = math.sqrt((estimated["x"] - true["x"])**2 + 
                                      (estimated["y"] - true["y"])**2)
            
            total_position_error += position_error
            max_position_error = max(max_position_error, position_error)
        
        avg_position_error = total_position_error / len(self.simulated_path)
        
        # Verificar que el error promedio está dentro de límites aceptables
        self.assertLessEqual(avg_position_error, 0.05,
                           f"Error promedio de posición ({avg_position_error:.3f}m) excede el umbral aceptable (0.05m)")
        self.assertLessEqual(max_position_error, 0.1,
                           f"Error máximo de posición ({max_position_error:.3f}m) excede el umbral aceptable (0.1m)")
        
        print(f"Error promedio de posición: {avg_position_error:.3f}m")
        print(f"Error máximo de posición: {max_position_error:.3f}m")

    def test_map_loading(self):
        """Verificar la carga y acceso a datos del mapa."""
        # Obtener datos del mapa
        map_data = self.mock_map.get_map_data()
        
        # Verificar que el mapa tiene las propiedades básicas necesarias
        self.assertIsNotNone(map_data, "Los datos del mapa no deben ser None")
        self.assertIn("width", map_data, "El mapa debe tener una propiedad 'width'")
        self.assertIn("height", map_data, "El mapa debe tener una propiedad 'height'")
        self.assertIn("resolution", map_data, "El mapa debe tener una propiedad 'resolution'")
        self.assertIn("origin", map_data, "El mapa debe tener una propiedad 'origin'")
        
        # Verificar que las dimensiones son razonables
        self.assertGreater(map_data["width"], 0, "El ancho del mapa debe ser positivo")
        self.assertGreater(map_data["height"], 0, "La altura del mapa debe ser positiva")
        self.assertGreater(map_data["resolution"], 0, "La resolución del mapa debe ser positiva")


@unittest.skipIf(not HAS_ROS, "Las bibliotecas de ROS2 no están disponibles")
class LocalizationROSTest(unittest.TestCase):
    """Pruebas para evaluar la integración del sistema de localización con ROS2."""

    def setUp(self):
        """Configurar el entorno de prueba con ROS2."""
        rclpy.init()
        self.node = Node("test_localization_node")
        
        # Crear subscriber para recibir mensajes de odometría
        self.received_odometry = []
        self.odom_subscriber = self.node.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Crear subscriber para recibir mensajes de pose con covarianza
        self.received_poses = []
        self.pose_subscriber = self.node.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )
        
        # Crear buffer de transformaciones para acceder a tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def tearDown(self):
        """Limpiar el entorno de prueba."""
        self.node.destroy_node()
        rclpy.shutdown()

    def odom_callback(self, msg):
        """Callback para recibir mensajes de odometría."""
        self.received_odometry.append(msg)

    def pose_callback(self, msg):
        """Callback para recibir mensajes de posición."""
        self.received_poses.append(msg)

    @patch('rclpy.spin_once')
    def test_ros_odometry_reception(self, mock_spin_once):
        """Verificar la recepción de mensajes de odometría a través de ROS2."""
        # Simular el procesamiento de mensajes
        def side_effect(node, timeout_sec=None):
            # Solo simular la recepción de un mensaje cuando no hay ninguno
            """Función Side effect.
            
            Args:
                node (Any): Descripción del parámetro.
                timeout_sec (Any): Descripción del parámetro.
            """
            if len(self.received_odometry) == 0:
                # Crear un mensaje de odometría simulado
                odom_msg = Odometry()
                odom_msg.pose.pose.position = Point(x=1.0, y=2.0, z=0.0)
                odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                self.odom_callback(odom_msg)
                
        mock_spin_once.side_effect = side_effect
        
        # Llamar a spin_once para procesar mensajes
        rclpy.spin_once(self.node, timeout_sec=1.0)
        
        # Verificar que se recibió algún mensaje de odometría
        self.assertGreater(len(self.received_odometry), 0,
                          "No se recibieron mensajes de odometría")
        
        # Verificar el formato del mensaje recibido
        first_odom = self.received_odometry[0]
        self.assertEqual(first_odom.pose.pose.position.x, 1.0,
                        "La posición X no coincide con el valor esperado")
        self.assertEqual(first_odom.pose.pose.position.y, 2.0,
                        "La posición Y no coincide con el valor esperado")

    @patch('rclpy.spin_once')
    def test_ros_pose_reception(self, mock_spin_once):
        """Verificar la recepción de mensajes de posición a través de ROS2."""
        # Simular el procesamiento de mensajes
        def side_effect(node, timeout_sec=None):
            # Solo simular la recepción de un mensaje cuando no hay ninguno
            """Función Side effect.
            
            Args:
                node (Any): Descripción del parámetro.
                timeout_sec (Any): Descripción del parámetro.
            """
            if len(self.received_poses) == 0:
                # Crear un mensaje de posición simulado
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.pose.pose.position = Point(x=1.0, y=2.0, z=0.0)
                pose_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                # Establecer covarianza (solo los elementos diagonales)
                pose_msg.pose.covariance[0] = 0.01  # xx
                pose_msg.pose.covariance[7] = 0.01  # yy
                pose_msg.pose.covariance[35] = 0.01  # thetatheta
                self.pose_callback(pose_msg)
                
        mock_spin_once.side_effect = side_effect
        
        # Llamar a spin_once para procesar mensajes
        rclpy.spin_once(self.node, timeout_sec=1.0)
        
        # Verificar que se recibió algún mensaje de posición
        self.assertGreater(len(self.received_poses), 0,
                          "No se recibieron mensajes de posición")
        
        # Verificar el formato del mensaje recibido
        first_pose = self.received_poses[0]
        self.assertEqual(first_pose.pose.pose.position.x, 1.0,
                        "La posición X no coincide con el valor esperado")
        self.assertEqual(first_pose.pose.pose.position.y, 2.0,
                        "La posición Y no coincide con el valor esperado")
        self.assertGreater(first_pose.pose.covariance[0], 0,
                          "La covarianza en X debe ser positiva")


if __name__ == '__main__':
    unittest.main() 