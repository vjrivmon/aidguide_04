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

# Clase que convierte imágenes de ROS2 a OpenCV y detecta coches con un clasificador Haar
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

        package_share_directory = get_package_share_directory('aidguide_04_capture_image')
        cascade_path = os.path.join(package_share_directory, 'clasificadores', 'haarcascade_car.xml')

        self.car_cascade = cv2.CascadeClassifier(cascade_path)

        if self.car_cascade.empty():
            print("Error: No se pudo cargar el clasificador Haar.")
            return

        # Carpeta donde se guardarán las imágenes (ruta absoluta)
        self.output_folder = '/home/irene/aidguide_04/aidguide_04_ws/src/aidguide_04_capture_image/aidguide_04_capture_image/fotos_detectadas'
        os.makedirs(self.output_folder, exist_ok=True)
        
        # Control de tiempo para capturar cada 3 segundos
        self.last_capture_time = 0
        self.capture_interval = 3.0  # segundos
        
    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print("Error al convertir la imagen:", e)
            return

        # Procesamiento para todos los frames (mostrar detección en tiempo real)
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        cars = self.car_cascade.detectMultiScale(
            gray_image,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        
        # Crear una copia de la imagen para mostrar y guardar
        image_with_detections = cv_image.copy()
        
        # Dibujar los rectángulos en todos los frames
        for (x, y, w, h) in cars:
            cv2.rectangle(image_with_detections, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(image_with_detections, 'Coche', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Verificar si es momento de guardar la imagen (cada 3 segundos)
        current_time = time.time()
        if current_time - self.last_capture_time >= self.capture_interval and len(cars) > 0:
            # Si hay coches y es momento de guardar, guardar la imagen con las detecciones
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.output_folder, f'coche_{timestamp}.jpg')
            cv2.imwrite(filename, image_with_detections)
            print(f"[INFO] Imagen guardada con detecciones: {filename}")
            self.last_capture_time = current_time
        
        # Mostrar la imagen con los rectángulos en tiempo real (siempre)
        cv2.imshow("Imagen capturada por el robot", image_with_detections)
        cv2.waitKey(1)  

def main(args=None):
    rclpy.init(args=args) 
    img_converter_object = Ros2OpenCVImageConverter()

    try:
        rclpy.spin(img_converter_object)  
    except KeyboardInterrupt:
        img_converter_object.destroy_node()
        print("Fin del programa!")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
