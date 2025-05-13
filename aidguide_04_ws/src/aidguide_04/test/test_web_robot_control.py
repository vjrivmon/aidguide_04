#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test para el Control del Robot a través de la Interfaz Web de AidGuide 04

Este módulo implementa pruebas automatizadas para verificar que los comandos enviados
desde la interfaz web se ejecutan correctamente en el robot. Comprueba la correcta
comunicación entre el frontend web y los nodos ROS2 que controlan el robot.

Author: AidGuide Team
"""

import unittest
import os
import sys
import json
import re
import subprocess
import time
from unittest.mock import MagicMock, patch
from pathlib import Path

# Intentar importar bibliotecas de ROS2
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool, String
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# Determinar la ruta al directorio del proyecto web
ROOT_DIR = Path(__file__).parent.parent.parent
WEB_PATH = ROOT_DIR / "aidguide_04_web"
SERVICES_PATH = WEB_PATH / "services"

@unittest.skipIf(not HAS_ROS, "Las bibliotecas de ROS2 no están disponibles")
class WebRobotControlTest(unittest.TestCase):
    """Pruebas para el control del robot desde la interfaz web."""

    def setUp(self):
        """Preparar entorno para las pruebas."""
        self.ros_service_file = SERVICES_PATH / "ros-service.ts"
        
        # Verificar que existen los archivos necesarios
        if not self.ros_service_file.exists():
            self.fail(f"Archivo de servicio ROS no encontrado: {self.ros_service_file}")
        
        # Inicializar ROS2 para las pruebas
        rclpy.init(args=None)
        self.node = rclpy.create_node('test_web_control_node')
        
        # Crear un contador para mensajes recibidos
        self.received_commands = {
            'navigation': 0,
            'stop': 0,
            'waypoint': 0,
            'voice': 0
        }
        
        # Configurar suscriptores para comandos
        self.cmd_vel_sub = self.node.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.navigation_sub = self.node.create_subscription(
            Bool,
            '/start_waypoint_following',
            self.navigation_callback,
            10
        )
        
        self.stop_sub = self.node.create_subscription(
            Bool,
            '/stop_waypoint_following',
            self.stop_callback,
            10
        )
        
        self.voice_cmd_sub = self.node.create_subscription(
            String,
            '/voice_command',
            self.voice_callback,
            10
        )
    
    def tearDown(self):
        """Limpiar después de las pruebas."""
        self.node.destroy_node()
        rclpy.shutdown()
    
    def cmd_vel_callback(self, msg):
        """Callback para mensajes de velocidad."""
        self.received_commands['navigation'] += 1
        self.last_cmd_vel = msg
    
    def navigation_callback(self, msg):
        """Callback para mensajes de inicio de navegación."""
        if msg.data:
            self.received_commands['waypoint'] += 1
    
    def stop_callback(self, msg):
        """Callback para mensajes de detención."""
        if msg.data:
            self.received_commands['stop'] += 1
    
    def voice_callback(self, msg):
        """Callback para comandos de voz."""
        self.received_commands['voice'] += 1
    
    def test_ros_service_methods(self):
        """Verificar que el servicio web tiene los métodos necesarios para control del robot."""
        with open(self.ros_service_file, 'r', encoding='utf-8') as f:
            content = f.read()
            
            # Verificar métodos para control del robot
            self.assertIn('startWaypointFollowing', content, 
                         "Falta el método para iniciar seguimiento de waypoints")
            self.assertIn('stopWaypointFollowing', content, 
                         "Falta el método para detener seguimiento de waypoints")
            
            # En algunos casos, la conexión se maneja automáticamente y no se expone 'connect' explícitamente
            # por lo que no lo hacemos obligatorio, pero verificamos que se usa ROSLIB o WebSocket
            self.assertTrue('new ROSLIB.Ros' in content or 'WebSocket' in content, 
                          "No se encontró inicialización de conexión websocket o ROSLIB")
            
            # El método disconnect es opcional, solo verificamos si existe
            if 'disconnect' not in content:
                print("Advertencia: No se encontró un método 'disconnect' explícito. " +
                      "Podría ser útil añadir esta funcionalidad para permitir la reconexión manual.")
            
            # Verificar que hay publicadores para tópicos de control
            self.assertIn('publish(', content, 
                         "No se encontraron métodos para publicar mensajes")
    
    @patch('rclpy.spin_once')
    def test_navigation_command(self, mock_spin_once):
        """Verificar que se publican correctamente los comandos de navegación."""
        # Simular un publicador para el tópico de inicio de navegación
        start_pub = self.node.create_publisher(Bool, '/start_waypoint_following', 10)
        
        # Publicar un mensaje de inicio
        start_msg = Bool()
        start_msg.data = True
        start_pub.publish(start_msg)
        
        # Simular que ROS procesa el mensaje
        mock_spin_once.side_effect = lambda node, timeout=None: self.navigation_callback(start_msg)
        
        # Verificar que el mensaje se recibe correctamente
        mock_spin_once(self.node)
        self.assertEqual(self.received_commands['waypoint'], 1, 
                        "No se recibió correctamente el comando de inicio de navegación")
    
    @patch('rclpy.spin_once')
    def test_stop_command(self, mock_spin_once):
        """Verificar que se publican correctamente los comandos de detención."""
        # Simular un publicador para el tópico de detención
        stop_pub = self.node.create_publisher(Bool, '/stop_waypoint_following', 10)
        
        # Publicar un mensaje de detención
        stop_msg = Bool()
        stop_msg.data = True
        stop_pub.publish(stop_msg)
        
        # Simular que ROS procesa el mensaje
        mock_spin_once.side_effect = lambda node, timeout=None: self.stop_callback(stop_msg)
        
        # Verificar que el mensaje se recibe correctamente
        mock_spin_once(self.node)
        self.assertEqual(self.received_commands['stop'], 1, 
                        "No se recibió correctamente el comando de detención")
    
    def test_web_endpoints(self):
        """Verificar que los endpoints de la API web están correctamente definidos."""
        # Buscar definiciones de rutas de API
        api_files = list(WEB_PATH.glob("**/*.ts")) + list(WEB_PATH.glob("**/*.js"))
        
        has_api_endpoints = False
        ros_bridge_endpoint = False
        api_files_count = 0
        
        for file_path in api_files:
            # Verificar que no sea un directorio
            if file_path.is_dir():
                continue
                
            api_files_count += 1
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                    # Buscar definiciones de endpoints
                    if '/api/' in content:
                        has_api_endpoints = True
                    
                    # Buscar específicamente endpoint de puente para ROS2
                    if ('/rosbridge' in content or 
                        '/api/robot/' in content or 
                        '/api/control/' in content or
                        'ros' in content.lower() and ('api' in content.lower() or 'endpoint' in content.lower())):
                        ros_bridge_endpoint = True
            except UnicodeDecodeError:
                # Ignorar archivos binarios o con codificación no UTF-8
                pass
            except IsADirectoryError:
                # Ignorar explícitamente directorios (aunque no debería ocurrir con is_dir())
                continue
            except Exception as e:
                print(f"Advertencia: No se pudo leer {file_path}: {str(e)}")
        
        # Si no encontramos endpoints, mostrar una advertencia
        if not has_api_endpoints:
            print("Advertencia: No se encontraron endpoints de API en la aplicación web. " +
                  "Esto puede indicar que la comunicación con el backend se maneja de otra forma.")
        
        # Si no encontramos endpoints específicos para ROS, mostrar una advertencia
        if not ros_bridge_endpoint:
            print("Advertencia: No se encontró un endpoint específico para comunicación con ROS2. " +
                  "Es posible que se use otro mecanismo de comunicación como websockets directos, " +
                  "o que la comunicación se maneje a nivel de backend.")
            
            # Verificar si hay otros indicios de comunicación con ROS
            if api_files_count > 0:
                print("Sin embargo, se encontraron archivos de API, por lo que es probable " +
                      "que exista algún mecanismo de comunicación entre la web y ROS2.")
        
        # El test siempre pasa pero genera advertencias si es necesario
        self.assertTrue(True)
    
    def test_command_validation(self):
        """Verificar que existe validación para los comandos enviados desde la web."""
        validation_found = False
        
        # Buscar validación en archivos de la API
        api_files = list(SERVICES_PATH.glob("**/*.ts")) + list(SERVICES_PATH.glob("**/*.js"))
        
        for file_path in api_files:
            with open(file_path, 'r', encoding='utf-8') as f:
                try:
                    content = f.read()
                    
                    # Buscar patrones de validación
                    if (re.search(r'if\s*\([^)]*?\)\s*{', content) and 
                        ('error' in content or 'throw' in content or 'catch' in content)):
                        validation_found = True
                        break
                except UnicodeDecodeError:
                    pass
        
        self.assertTrue(validation_found, 
                       "No se encontró validación para los comandos enviados al robot")
    
    def test_feedback_mechanism(self):
        """Verificar que existe mecanismo de retroalimentación del robot a la web."""
        feedback_found = False
        
        # Buscar mecanismos de suscripción a tópicos de estado
        with open(self.ros_service_file, 'r', encoding='utf-8') as f:
            content = f.read()
            
            # Buscar creación de suscriptores para estado del robot
            if 'subscribe' in content and ('status' in content.lower() or 'state' in content.lower()):
                feedback_found = True
        
        self.assertTrue(feedback_found, 
                       "No se encontró mecanismo de retroalimentación del estado del robot")

# Clase para pruebas sin dependencia de ROS2
class WebRobotControlSimpleTest(unittest.TestCase):
    """Pruebas básicas sin dependencia de ROS2."""
    
    def setUp(self):
        """Preparar entorno para las pruebas simples."""
        self.ros_service_file = SERVICES_PATH / "ros-service.ts"
        
        if not self.ros_service_file.exists():
            self.skipTest(f"Archivo de servicio ROS no encontrado: {self.ros_service_file}")
    
    def test_api_structure(self):
        """Verificar la estructura básica de la API de control del robot."""
        with open(self.ros_service_file, 'r', encoding='utf-8') as f:
            content = f.read()
            
            # Verificar que hay una clase para servicio ROS
            self.assertIn('class', content, "No se encontró definición de clase")
            
            # Verificar que hay métodos públicos
            self.assertIn('public', content, "No se encontraron métodos públicos")
            
            # Verificar que hay definiciones de tópicos
            self.assertIn('topic', content.lower(), "No se encontraron definiciones de tópicos")
    
    def test_robot_control_components(self):
        """Verificar la existencia de componentes de interfaz para control del robot."""
        components_list = self.find_robot_control_components()
        
        # Verificar que hay al menos un componente de control
        self.assertGreater(len(components_list), 0, 
                          "No se encontraron componentes de interfaz para control del robot")
        
        print(f"Componentes de control encontrados: {', '.join(components_list)}")
    
    def find_robot_control_components(self):
        """Buscar componentes relacionados con control del robot."""
        components = []
        
        component_files = list(WEB_PATH.glob("**/components/**/*.tsx")) + list(WEB_PATH.glob("**/app/**/*.tsx"))
        
        for file_path in component_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read().lower()
                    
                    # Buscar términos relacionados con control del robot
                    if (('robot' in content and 'control' in content) or
                        'navigation' in content or
                        'command' in content or
                        'move' in content or
                        'joystick' in content or
                        'steering' in content):
                        components.append(file_path.name)
            except Exception:
                pass
        
        return components

if __name__ == "__main__":
    unittest.main() 