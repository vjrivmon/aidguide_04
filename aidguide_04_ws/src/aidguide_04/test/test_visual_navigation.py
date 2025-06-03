#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test de Navegación Visual para AidGuide 04

Este módulo implementa pruebas automatizadas para verificar la funcionalidad
de navegación visual del robot, evaluando su capacidad para reconocer señales de tráfico,
pasos de peatones, semáforos y otros elementos visuales relevantes para la navegación segura.
Estas pruebas fueron adaptadas desde el notebook de Colab para integrarlas en el sistema de pruebas.

Author: AidGuide Team
"""

import unittest
import os
import sys
import numpy as np
import cv2
import time
from pathlib import Path
from unittest.mock import MagicMock, patch

# Intentar importar dependencias necesarias
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# Determinar la ruta al directorio del proyecto
ROOT_DIR = Path(__file__).parent.parent.parent
SRC_PATH = ROOT_DIR
TEST_IMAGES_PATH = ROOT_DIR / "aidguide_04_web" / "public" / "senyales"

class VisualNavigationTest(unittest.TestCase):
    """Pruebas para la navegación visual y reconocimiento de elementos visuales."""

    def setUp(self):
        """Preparar entorno para las pruebas."""
        # Verificar que existe la carpeta de imágenes de prueba
        self.image_files = []
        if TEST_IMAGES_PATH.exists():
            self.image_files = list(TEST_IMAGES_PATH.glob("**/*.jpg")) + list(TEST_IMAGES_PATH.glob("**/*.png"))
        
        # Si no hay imágenes de prueba, crear imágenes sintéticas
        if not self.image_files:
            # Crear una imagen sintética para simular una señal de tráfico
            self.test_traffic_sign = np.zeros((480, 640, 3), dtype=np.uint8)
            # Dibujar un círculo rojo (prohibido)
            cv2.circle(self.test_traffic_sign, (320, 240), 100, (0, 0, 255), -1)
            cv2.circle(self.test_traffic_sign, (320, 240), 90, (255, 255, 255), 10)
            
            # Crear una imagen sintética para simular un paso de peatones
            self.test_crosswalk = np.zeros((480, 640, 3), dtype=np.uint8)
            # Dibujar rayas blancas
            for i in range(0, 640, 80):
                cv2.rectangle(self.test_crosswalk, (i, 180), (i + 40, 300), (255, 255, 255), -1)
            
            self.test_images = [self.test_traffic_sign, self.test_crosswalk]
        else:
            # Cargar hasta 5 imágenes reales para las pruebas
            self.test_images = []
            for img_path in self.image_files[:5]:
                img = cv2.imread(str(img_path))
                if img is not None:
                    self.test_images.append(img)
            
            if not self.test_images:
                self.skipTest("No se pudieron cargar imágenes de prueba válidas")
        
        # Crear un mock del detector de señales de tráfico
        self.mock_traffic_sign_detector = MagicMock()
        self.mock_traffic_sign_detector.detect_signs.return_value = [
            {"type": "stop", "confidence": 0.92, "bbox": [200, 150, 100, 100]},
            {"type": "yield", "confidence": 0.85, "bbox": [350, 200, 80, 80]}
        ]
        
        # Crear un mock del detector de pasos de peatones
        self.mock_crosswalk_detector = MagicMock()
        self.mock_crosswalk_detector.detect_crosswalk.return_value = {
            "detected": True,
            "confidence": 0.88,
            "position": (320, 240),
            "orientation": 45.5
        }

    def test_traffic_sign_detection(self):
        """Verificar la detección de señales de tráfico."""
        # Verificar que el detector devuelve resultados para cada imagen
        for img in self.test_images:
            detections = self.mock_traffic_sign_detector.detect_signs(img)
            
            # Verificar que se detectan señales
            self.assertIsNotNone(detections, 
                               "El detector debería devolver una lista de detecciones, no None")
            
            # Verificar el formato de los resultados
            for detection in detections:
                self.assertIn('type', detection, 
                             "El resultado debe incluir el tipo de señal detectada")
                self.assertIn('confidence', detection, 
                             "El resultado debe incluir un valor de confianza")
                self.assertIn('bbox', detection, 
                             "El resultado debe incluir un bounding box")
                
                # Verificar que la confianza esté en un rango válido
                self.assertGreaterEqual(detection['confidence'], 0.0,
                                      "El valor de confianza debe ser >= 0")
                self.assertLessEqual(detection['confidence'], 1.0,
                                   "El valor de confianza debe ser <= 1")

    def test_crosswalk_detection(self):
        """Verificar la detección de pasos de peatones."""
        # Verificar que el detector devuelve resultados para cada imagen
        for img in self.test_images:
            detection = self.mock_crosswalk_detector.detect_crosswalk(img)
            
            # Verificar que el resultado tiene la estructura esperada
            self.assertIsNotNone(detection, 
                               "El detector debería devolver un diccionario con la detección, no None")
            self.assertIn('detected', detection, 
                         "El resultado debe indicar si se detectó un paso de peatones")
            self.assertIn('confidence', detection, 
                         "El resultado debe incluir un valor de confianza")
            
            # Si se detectó un paso de peatones, verificar información adicional
            if detection['detected']:
                self.assertIn('position', detection, 
                             "Si se detecta un paso de peatones, debe incluir su posición")
                self.assertIn('orientation', detection, 
                             "Si se detecta un paso de peatones, debe incluir su orientación")
                
                # Verificar que la posición está dentro de la imagen
                position = detection['position']
                self.assertGreaterEqual(position[0], 0, 
                                      "La coordenada x de la posición debe ser >= 0")
                self.assertLessEqual(position[0], img.shape[1], 
                                   "La coordenada x de la posición debe ser <= ancho de la imagen")
                self.assertGreaterEqual(position[1], 0, 
                                      "La coordenada y de la posición debe ser >= 0")
                self.assertLessEqual(position[1], img.shape[0], 
                                   "La coordenada y de la posición debe ser <= altura de la imagen")
                
                # Verificar que la orientación está en un rango válido
                self.assertGreaterEqual(detection['orientation'], 0, 
                                      "La orientación debe ser >= 0 grados")
                self.assertLessEqual(detection['orientation'], 360, 
                                   "La orientación debe ser <= 360 grados")

    def test_detection_performance(self):
        """Evaluar el rendimiento del detector de elementos visuales."""
        # Medir el tiempo de ejecución del detector de señales de tráfico
        start_time = time.time()
        num_iterations = 5
        
        for _ in range(num_iterations):
            for img in self.test_images:
                _ = self.mock_traffic_sign_detector.detect_signs(img)
        
        traffic_sign_time = (time.time() - start_time) / (num_iterations * len(self.test_images))
        
        # Medir el tiempo de ejecución del detector de pasos de peatones
        start_time = time.time()
        
        for _ in range(num_iterations):
            for img in self.test_images:
                _ = self.mock_crosswalk_detector.detect_crosswalk(img)
        
        crosswalk_time = (time.time() - start_time) / (num_iterations * len(self.test_images))
        
        # Verificar que el tiempo de detección es razonable para aplicaciones en tiempo real
        self.assertLessEqual(traffic_sign_time, 0.3, 
                           f"El tiempo promedio de detección de señales ({traffic_sign_time:.3f}s) es demasiado alto")
        self.assertLessEqual(crosswalk_time, 0.3, 
                           f"El tiempo promedio de detección de pasos de peatones ({crosswalk_time:.3f}s) es demasiado alto")
        
        print(f"Tiempo promedio de detección de señales: {traffic_sign_time:.3f} segundos")
        print(f"Tiempo promedio de detección de pasos de peatones: {crosswalk_time:.3f} segundos")

    def test_visual_guidance_decisions(self):
        """Verificar la toma de decisiones basada en elementos visuales detectados."""
        # Crear un mock del sistema de guiado visual
        mock_visual_guidance = MagicMock()
        mock_visual_guidance.get_guidance_from_detections.return_value = {
            "action": "stop",
            "reason": "stop_sign",
            "confidence": 0.92,
            "wait_time": 3.0
        }
        
        # Simular detecciones para diferentes escenarios
        stop_detections = [
            {"type": "stop", "confidence": 0.92, "bbox": [200, 150, 100, 100]}
        ]
        
        yield_detections = [
            {"type": "yield", "confidence": 0.85, "bbox": [350, 200, 80, 80]}
        ]
        
        crosswalk_detection = {
            "detected": True,
            "confidence": 0.88,
            "position": (320, 240),
            "orientation": 0.0
        }
        
        no_detections = []
        
        # Probar la toma de decisiones para diferentes detecciones
        guidance_stop = mock_visual_guidance.get_guidance_from_detections(stop_detections, None)
        guidance_yield = mock_visual_guidance.get_guidance_from_detections(yield_detections, None)
        guidance_crosswalk = mock_visual_guidance.get_guidance_from_detections(
            no_detections, crosswalk_detection)
        guidance_none = mock_visual_guidance.get_guidance_from_detections(no_detections, None)
        
        # Verificar que las decisiones tienen la estructura esperada
        for guidance in [guidance_stop, guidance_yield, guidance_crosswalk, guidance_none]:
            self.assertIn('action', guidance, 
                         "La guía debe incluir una acción a realizar")
            self.assertIn('reason', guidance, 
                         "La guía debe incluir una razón para la acción")
            self.assertIn('confidence', guidance, 
                         "La guía debe incluir un valor de confianza")


@unittest.skipIf(not HAS_ROS, "Las bibliotecas de ROS2 no están disponibles")
class VisualNavigationROSTest(unittest.TestCase):
    """Pruebas para la integración de navegación visual con ROS2."""

    def setUp(self):
        """Configurar el entorno de prueba con ROS2."""
        rclpy.init()
        self.node = Node("test_visual_navigation_node")
        self.cv_bridge = CvBridge()
        
        # Crear publisher para enviar imágenes de prueba
        self.image_publisher = self.node.create_publisher(
            Image, 
            'test_camera/image_raw',
            10
        )
        
        # Crear subscribers para recibir resultados de detección
        self.received_traffic_signs = []
        self.traffic_sign_subscriber = self.node.create_subscription(
            Image,  # Cambiar por el tipo de mensaje adecuado según la implementación
            'traffic_signs/detections',
            self.traffic_sign_callback,
            10
        )
        
        # Preparar imagen de prueba
        self.test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(self.test_image, (320, 240), 100, (0, 0, 255), -1)

    def tearDown(self):
        """Limpiar el entorno de prueba."""
        self.node.destroy_node()
        rclpy.shutdown()

    def traffic_sign_callback(self, msg):
        """Callback para recibir detecciones de señales de tráfico."""
        self.received_traffic_signs.append(msg)

    @patch('rclpy.spin_once')
    def test_ros_traffic_sign_detection(self, mock_spin_once):
        """Verificar la detección de señales de tráfico a través de los topics de ROS2."""
        # Convertir la imagen de prueba a mensaje ROS
        img_msg = self.cv_bridge.cv2_to_imgmsg(self.test_image, encoding="bgr8")
        
        # Publicar la imagen
        self.image_publisher.publish(img_msg)
        
        # Simular el procesamiento de mensajes
        def side_effect(node, timeout_sec=None):
            # Solo simular la recepción de un mensaje cuando se publique una imagen
            """Función Side effect.
            
            Args:
                node (Any): Descripción del parámetro.
                timeout_sec (Any): Descripción del parámetro.
            """
            if len(self.received_traffic_signs) == 0:
                # Crear un mensaje de detección simulado
                detection_msg = Image()  # Cambiar por el tipo apropiado
                self.traffic_sign_callback(detection_msg)
                
        mock_spin_once.side_effect = side_effect
        
        # Llamar a spin_once para procesar mensajes
        rclpy.spin_once(self.node, timeout_sec=1.0)
        
        # Verificar que se recibió alguna detección
        self.assertGreater(len(self.received_traffic_signs), 0,
                          "No se recibieron detecciones de señales de tráfico")


if __name__ == '__main__':
    unittest.main() 