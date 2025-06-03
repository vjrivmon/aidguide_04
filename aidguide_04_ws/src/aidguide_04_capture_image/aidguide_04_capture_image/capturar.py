#!/usr/bin/env python3

import rclpy  
import cv2  
import numpy as np
import os
import time
from ament_index_python.packages import get_package_share_directory  
from cv_bridge import CvBridge, CvBridgeError  
from sensor_msgs.msg import Image 
from rclpy.node import Node  
from rclpy.qos import ReliabilityPolicy, QoSProfile 
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
from datetime import datetime

# Clase que convierte imágenes de ROS2 a OpenCV y detecta objetos (coches, caras, señales stop, peatones y semáforos) con clasificadores Haar
class Ros2OpenCVImageConverter(Node):   

    """Clase Ros2OpenCVImageConverter.
    
    Implementa funcionalidad para Ros2OpenCVImageConverter.
    """
    def __init__(self):
        """Función Constructor.
        """
        super().__init__('Ros2OpenCVImageConverter')  

        self.bridge_object = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/image', 
            self.camera_callback,  
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)  
        )

        # Cargar clasificadores
        package_share_directory = get_package_share_directory('aidguide_04_capture_image')
        
        # Clasificador para coches
        car_cascade_path = os.path.join(package_share_directory, 'clasificadores', 'haarcascade_car.xml')
        self.car_cascade = cv2.CascadeClassifier(car_cascade_path)
        if self.car_cascade.empty():
            self.get_logger().error("Error: No se pudo cargar el clasificador de coches.")
            return

        # Clasificador para caras
        face_cascade_path = os.path.join(package_share_directory, 'clasificadores', 'haarcascade_frontalface_default.xml')
        self.face_cascade = cv2.CascadeClassifier(face_cascade_path)
        if self.face_cascade.empty():
            self.get_logger().error("Error: No se pudo cargar el clasificador de caras.")
            return


        # Clasificador para stop
        stop_cascade_path = os.path.join(package_share_directory, 'clasificadores', 'cascade_stop_3_15.xml')
        self.stop_cascade= cv2.CascadeClassifier(stop_cascade_path)
        if self.stop_cascade.empty():
            self.get_logger().error("Error: No se pudo cargar el clasificador de stops.")
            return
        
        # Clasificador para pesh
        #pesh_cascade_path = os.path.join(package_share_directory, 'clasificadores', 'cascade_pesh_lbp14.xml')
        #self.pesh_cascade = cv2.CascadeClassifier(pesh_cascade_path)
        #if self.pesh_cascade.empty():
         #   self.get_logger().error("Error: No se pudo cargar el clasificador de personas.")
          #  return
        
        # Clasificador para semáforos
        z_cascade_path = os.path.join(package_share_directory, 'clasificadores', 'cascade_zapr_lbp13.xml')
        self.z_cascade = cv2.CascadeClassifier(z_cascade_path)
        if self.z_cascade.empty():
            self.get_logger().error("Error: No se pudo cargar el clasificador de semáforos.")
            return
        

        # Clasificador para semáforos
        traffic_light_cascade_path = os.path.join(package_share_directory, 'clasificadores', 'TrafficLight_HAAR_16Stages.xml')
        self.traffic_light_cascade = cv2.CascadeClassifier(traffic_light_cascade_path)
        if self.traffic_light_cascade.empty():
            self.get_logger().error("Error: No se pudo cargar el clasificador de semáforos.")
            return

        # Carpeta donde se guardarán las imágenes
        self.output_folder = os.path.join(os.path.expanduser('~'), 'aidguide_04', 'aidguide_04_ws', 'src', 'aidguide_04_capture_image', 'aidguide_04_capture_image', 'fotos_detectadas')
        os.makedirs(self.output_folder, exist_ok=True)
        
        # Control de tiempo para capturas
        self.last_capture_time = 0
        self.capture_interval = 3.0  # segundos entre capturas
        
        self.get_logger().info('Nodo de detección inicializado correctamente')
        
    def camera_callback(self, msg):
        """Función Camera callback.
        
        Args:
            msg (Any): Descripción del parámetro.
        """
        try:
            # Convertir la imagen de ROS a OpenCV
            cv_image = self.bridge_object.imgmsg_to_cv2(msg, "bgr8")
            
            # Convertir a escala de grises
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detectar coches
            cars = self.car_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            # Detectar caras
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            # Detectar señales de stop
            stops = self.stop_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            # Detectar peatones
            #peshes = self.pesh_cascade.detectMultiScale(
             #   gray,
               # scaleFactor=1.1,
              #  minNeighbors=5,
                #minSize=(30, 30)
            #)
            
            # Detectar semáforos
            traffic_lights = self.traffic_light_cascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30)
            )
            
            # Dibujar rectángulos alrededor de los coches (azul)
            for (x, y, w, h) in cars:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(cv_image, 'Coche', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
            
            # Dibujar rectángulos alrededor de las caras (verde)
            for (x, y, w, h) in faces:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(cv_image, 'Cara', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # Dibujar rectángulos alrededor de las señales de stop (rojo)
            for (x, y, w, h) in stops:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(cv_image, 'Stop', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            
            # Dibujar rectángulos alrededor de los peatones (amarillo)
            #for (x, y, w, h) in peshes:
             #   cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 255), 2)
              #  cv2.putText(cv_image, 'Peaton', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
            
            # Dibujar rectángulos alrededor de los semáforos (morado)
            for (x, y, w, h) in traffic_lights:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 255), 2)
                cv2.putText(cv_image, 'Semaforo', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
            
            # Guardar imagen si se detectó algo y han pasado suficientes segundos
            current_time = time.time()
            if (len(cars) > 0 or len(faces) > 0 or len(stops) > 0 or len(traffic_lights) > 0) and current_time - self.last_capture_time >= self.capture_interval:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = os.path.join(self.output_folder, f'deteccion_{timestamp}.jpg')
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'Imagen guardada: {filename}')
                self.last_capture_time = current_time
            
            # Mostrar la imagen
            cv2.imshow('Detector de Objetos', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error en el procesamiento de imagen: {str(e)}')

def main(args=None):
    """Función Main.
    
    Args:
        args (Any): Descripción del parámetro.
    """
    rclpy.init(args=args)
    image_converter = Ros2OpenCVImageConverter()
    
    try:
        rclpy.spin(image_converter)
    except KeyboardInterrupt:
        pass
    finally:
        image_converter.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()