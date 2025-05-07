import rclpy  
import cv2  
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory  
from cv_bridge import CvBridge, CvBridgeError  
from sensor_msgs.msg import Image 
from rclpy.node import Node  
from rclpy.qos import ReliabilityPolicy, QoSProfile 

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
            print("Error: No se pudo cargar el clasificador Haar. Asegúrate de que haarcascade_car.xml está en la carpeta correcta.")
            return
        
    def camera_callback(self, data):
        try:
            # Convertimos la imagen de ROS a formato OpenCV
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