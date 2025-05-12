import rclpy  
import cv2  
import numpy as np
import os
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

        # Carpeta donde se guardarán las imágenes
        self.output_folder = os.path.join(package_share_directory, 'fotos_detectadas')
        os.makedirs(self.output_folder, exist_ok=True)
        
    def camera_callback(self, data):
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print("Error al convertir la imagen:", e)
            return

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        cars = self.car_cascade.detectMultiScale(
            gray_image,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )

        if len(cars) > 0:
            # Si hay coches, guardar la imagen
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(self.output_folder, f'coche_{timestamp}.jpg')
            cv2.imwrite(filename, cv_image)
            print(f"[INFO] Imagen guardada: {filename}")

        # Mostrar la imagen con los rectángulos
        for (x, y, w, h) in cars:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, 'Coche', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.imshow("Imagen capturada por el robot", cv_image)
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
