import rclpy
import cv2
import numpy as np
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from ament_index_python.packages import get_package_share_directory

class Ros2OpenCVImageConverter(Node):

    def __init__(self):
        # Inicializa el nodo con el nombre 'Ros2OpenCVImageConverter'
        super().__init__('Ros2OpenCVImageConverter')
        
        # Crea un objeto CvBridge para convertir mensajes ROS a imágenes OpenCV
        self.bridge_object = CvBridge()
        
        # Obtiene la ruta al directorio de recursos del paquete instalado
        package_share_dir = get_package_share_directory('aidguide_04_capture_image')
        classifier_path = os.path.join(package_share_dir, 'clasificadores', 'haarcascade_fullbody.xml')
        
        # Carga el clasificador para detección de cuerpo completo
        self.fullbody_cascade = cv2.CascadeClassifier(classifier_path)
        if self.fullbody_cascade.empty():
            self.get_logger().error(f"Error: No se pudo cargar el clasificador en {classifier_path}")
            raise RuntimeError("Fallo al cargar el clasificador de cuerpo completo")
        
        # Crea una suscripción al tópico de la cámara
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
    
    def camera_callback(self, data):
        try:
            # Convierte el mensaje de imagen ROS a formato OpenCV (bgr8)
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Error de CvBridge: {e}")
            return

        # Convierte la imagen a escala de grises para la detección
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detecta personas en la imagen usando el clasificador
        persons = self.fullbody_cascade.detectMultiScale(img_gray, scaleFactor=1.1, minNeighbors=5)
        
        # Dibuja un rectángulo alrededor de cada persona detectada
        for (x, y, w, h) in persons:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            # Extrae la región de interés (ROI)
            roi_color = cv_image[y:y+h, x:x+w]
            # Guarda la imagen con las detecciones
            cv2.imwrite('persona_detectada.jpg', cv_image)
            self.get_logger().info("Persona detectada y imagen guardada como persona_detectada.jpg")

        # Muestra la imagen en una ventana
        cv2.imshow("Imagen capturada por el robot", cv_image)
        cv2.waitKey(1)    

def main(args=None):
    # Inicializa el entorno ROS2
    rclpy.init(args=args)
    try:
        # Crea una instancia del nodo
        img_converter_object = Ros2OpenCVImageConverter()
        # Mantiene el nodo en ejecución
        rclpy.spin(img_converter_object)
    except RuntimeError as e:
        print(f"Error en la inicialización: {e}")
    except KeyboardInterrupt:
        # Cierra el nodo al interrumpir (Ctrl+C)
        img_converter_object.destroy_node()
        img_converter_object.get_logger().info("¡Fin del programa!")
    
    # Cierra todas las ventanas de OpenCV
    cv2.destroyAllWindows() 

if __name__ == '__main__':
    main()