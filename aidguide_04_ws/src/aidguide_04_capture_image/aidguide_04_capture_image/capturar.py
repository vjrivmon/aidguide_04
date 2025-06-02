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
from datetime import datetime

# Clase que convierte imágenes de ROS2 a OpenCV y detecta coches y caras con un clasificador Haar
class Ros2OpenCVImageConverter(Node):   

    def __init__(self):
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

        # Carpeta donde se guardarán las imágenes
        self.output_folder = os.path.join(os.path.expanduser('~'), 'aidguide_04', 'aidguide_04_ws', 'src', 'aidguide_04_capture_image', 'aidguide_04_capture_image', 'fotos_detectadas')
        os.makedirs(self.output_folder, exist_ok=True)
        
        # Control de tiempo para capturas
        self.last_capture_time = 0
        self.capture_interval = 3.0  # segundos entre capturas
        
        self.get_logger().info('Nodo de detección inicializado correctamente')
        
    def camera_callback(self, msg):
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
            
            # Dibujar rectángulos alrededor de los coches (azul)
            for (x, y, w, h) in cars:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(cv_image, 'Coche', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
            
            # Dibujar rectángulos alrededor de las caras (verde)
            for (x, y, w, h) in faces:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(cv_image, 'Cara', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # Guardar imagen si se detectó algo y han pasado suficientes segundos
            current_time = time.time()
            if (len(cars) > 0 or len(faces) > 0) and current_time - self.last_capture_time >= self.capture_interval:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = os.path.join(self.output_folder, f'deteccion_{timestamp}.jpg')
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'Imagen guardada: {filename}')
                self.last_capture_time = current_time
            
            # Mostrar la imagen
            cv2.imshow('Detector de Coches y Caras', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error en el procesamiento de imagen: {str(e)}')

def main(args=None):
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
