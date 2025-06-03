"""Nodo de ROS2 para la detección de frutas utilizando un modelo YOLOv8 TFLite.

Este nodo se suscribe a un tema de imágenes, procesa cada imagen utilizando
un modelo de detección de objetos TensorFlow Lite (YOLOv8) y publica
los resultados de la detección.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String # Ejemplo, podrías usar un mensaje customizado para las detecciones
import cv2
from cv_bridge import CvBridge
import numpy as np
import tensorflow as tf # O tflite_runtime.interpreter si usas solo el runtime
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Constantes (ajusta según sea necesario)
# MODEL_FILENAME se determinará dinámicamente ahora
INPUT_IMAGE_TOPIC = '/image' # Tema al que te suscribes para recibir imágenes
DETECTION_OUTPUT_TOPIC = '/detected_fruits' # Tema donde publicas las detecciones
CONFIDENCE_THRESHOLD = 0.5 # Umbral de confianza para considerar una detección

MODEL_BASENAME = 'best' # Nombre base de tus modelos
PREFERRED_PRECISION = 'float16'
FALLBACK_PRECISION = 'float32'

class FruitDetectorNode(Node):
    """Clase que representa el nodo de detección de frutas.

    Se encarga de cargar el modelo TFLite, suscribirse a un flujo de imágenes,
    realizar inferencias y publicar los resultados.
    """
    def __init__(self):
        """Constructor del nodo FruitDetectorNode.

        Inicializa el intérprete de TFLite, los suscriptores y publicadores de ROS2.
        Intenta cargar el modelo con la precisión preferida (float16), y si no,
        recurre a la precisión de fallback (float32).
        """
        super().__init__('fruit_detector_node')
        self.get_logger().info(f'Nodo {self.get_name()} inicializado.')

        package_base_path = get_package_share_directory('aidguide_04_deep_learning')
        resource_path = os.path.join(package_base_path, 'resource')

        preferred_model_filename = f"{MODEL_BASENAME}_{PREFERRED_PRECISION}.tflite"
        fallback_model_filename = f"{MODEL_BASENAME}_{FALLBACK_PRECISION}.tflite"

        model_to_load_path = None
        loaded_model_name = None

        # Intentar cargar el modelo con precisión preferida
        path_preferred = os.path.join(resource_path, preferred_model_filename)
        if os.path.exists(path_preferred):
            model_to_load_path = path_preferred
            loaded_model_name = preferred_model_filename
            self.get_logger().info(f"Intentando cargar modelo preferido: {preferred_model_filename}")
        else:
            self.get_logger().warning(
                f"Modelo preferido ({preferred_model_filename}) no encontrado en {resource_path}. "
                f"Intentando con modelo de fallback: {fallback_model_filename}."
            )
            path_fallback = os.path.join(resource_path, fallback_model_filename)
            if os.path.exists(path_fallback):
                model_to_load_path = path_fallback
                loaded_model_name = fallback_model_filename
                self.get_logger().info(f"Intentando cargar modelo de fallback: {fallback_model_filename}")
            else:
                self.get_logger().error(
                    f"Modelo de fallback ({fallback_model_filename}) tampoco encontrado en {resource_path}."
                )
                self.get_logger().error("No se pudo encontrar ningún modelo TFLite. El nodo no funcionará.")
                return # Salir si no hay modelo

        if model_to_load_path:
            try:
                self.interpreter = tf.lite.Interpreter(model_path=model_to_load_path)
                self.interpreter.allocate_tensors()
                self.get_logger().info(f'Modelo TFLite "{loaded_model_name}" cargado exitosamente desde: {model_to_load_path}')
            except Exception as e:
                self.get_logger().error(f"Error al cargar el modelo TFLite ({loaded_model_name}): {e}")
                return # Salir si la carga falla
        else:
            # Esto no debería ocurrir si la lógica anterior es correcta, pero por si acaso.
            self.get_logger().error("No se seleccionó ningún modelo para cargar. El nodo no funcionará.")
            return

        # Obtener detalles de entrada y salida del modelo
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.get_logger().info(f"Detalles de entrada del modelo: {self.input_details}")
        self.get_logger().info(f"Detalles de salida del modelo: {self.output_details}")

        # CvBridge para convertir entre imágenes de ROS y OpenCV
        self.bridge = CvBridge()

        # Suscriptor al tema de imágenes
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.image_subscription = self.create_subscription(
            Image,
            INPUT_IMAGE_TOPIC,
            self.image_callback,
            qos_profile
        )
        self.get_logger().info(f'Suscrito al tema de imágenes: {INPUT_IMAGE_TOPIC}')

        # Publicador para los resultados de la detección
        self.detection_publisher = self.create_publisher(
            String, 
            DETECTION_OUTPUT_TOPIC,
            10 # QoS profile depth
        )
        self.get_logger().info(f'Publicando detecciones en el tema: {DETECTION_OUTPUT_TOPIC}')

    def image_callback(self, msg: Image):
        """Callback que se ejecuta cada vez que se recibe una nueva imagen.

        Args:
            msg (sensor_msgs.msg.Image): El mensaje de imagen recibido.
        """
        self.get_logger().info(f'✨ Recibida imagen con timestamp: {msg.header.stamp}')

        try:
            # Convertir mensaje de imagen de ROS a formato OpenCV (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen de ROS a OpenCV: {e}")
            return

        # Preprocesamiento de la imagen (ajustar según los requisitos de tu modelo YOLOv8)
        # Esto es un ejemplo general, YOLOv8 podría requerir normalización específica,
        # cambio de tamaño manteniendo la relación de aspecto, padding, etc.
        input_shape = self.input_details[0]['shape'] # Ej: [1, height, width, 3]
        # Asegúrate de que el tamaño de entrada coincida con el esperado por el modelo.
        # El entrenamiento se hizo con imgsz=100 para el dataset fruits-360
        # Si tu modelo YOLO espera 640x640, por ejemplo:
        # target_height = input_shape[1]
        # target_width = input_shape[2]
        # resized_image = cv2.resize(cv_image, (target_width, target_height))
        
        # Para el modelo entrenado con 'yolov8n-cls.pt' y imgsz=100 en tu script de Colab:
        # El modelo de clasificación espera una imagen de un tamaño específico,
        # por ejemplo, 100x100 si imgsz=100.
        # Para detección de objetos, YOLO usualmente requiere un tamaño como 640x640.
        # Debes verificar qué espera tu modelo .tflite exportado (si es de clasificación o detección).
        # El script de Colab parece entrenar un CLASIFICADOR ('yolov8n-cls.pt').
        # Si es un clasificador, el output será una clase, no bounding boxes.
        # Si exportaste un modelo de DETECCIÓN, el preprocesamiento y postprocesamiento serán diferentes.

        # Suponiendo que es un modelo de CLASIFICACIÓN y espera 100x100:
        img_height = self.input_details[0]['shape'][1]
        img_width = self.input_details[0]['shape'][2]
        
        # Redimensionar la imagen al tamaño esperado por el modelo
        input_image = cv2.resize(cv_image, (img_width, img_height))
        
        # Convertir a RGB si es necesario (OpenCV usa BGR por defecto)
        # input_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2RGB) # Depende del modelo
        
        # Expandir dimensiones para que coincida con la forma de entrada del batch (1, H, W, C)
        input_data = np.expand_dims(input_image, axis=0)
        
        # Normalizar los valores de los píxeles si el modelo lo espera (e.g., 0-1 o -1 a 1)
        # Si tu modelo fue entrenado con imágenes normalizadas (divididas por 255.0, por ejemplo)
        # input_data = np.float32(input_data) / 255.0
        
        # Verificar el tipo de dato esperado por el modelo (usualmente float32 o uint8)
        if self.input_details[0]['dtype'] == np.float32:
            input_data = np.float32(input_data)
            # Normalizar si es float32 y el modelo lo espera (0-1)
            # input_data = input_data / 255.0 # Descomenta si es necesario
        elif self.input_details[0]['dtype'] == np.uint8:
            # No se necesita conversión si ya es uint8 y no hay normalización.
            pass


        # Establecer el tensor de entrada
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)

        # Ejecutar la inferencia
        self.interpreter.invoke()

        # Obtener los resultados de la inferencia
        # La estructura de 'output_data' dependerá de si es un modelo de clasificación o detección.
        # Para CLASIFICACIÓN:
        output_data_cls = self.interpreter.get_tensor(self.output_details[0]['index'])
        # output_data_cls será un array de scores para cada clase, ej: [[0.1, 0.8, 0.05, ...]]
        predicted_class_index = np.argmax(output_data_cls[0])
        confidence_score = output_data_cls[0][predicted_class_index]

        # También obtener las 3 mejores detecciones
        top_indices = np.argsort(output_data_cls[0])[-3:][::-1]
        top_scores = output_data_cls[0][top_indices]

        # Aquí necesitarás un mapeo de índices a nombres de clases (frutas)
        # Este mapeo deberías obtenerlo de tu proceso de entrenamiento.
        class_names = [
            "Apple 10", "Apple 11", "Apple 12", "Apple 13", "Apple 14", "Apple 17", "Apple 18", "Apple 19",
            "Apple 5", "Apple 6", "Apple 7", "Apple 8", "Apple 9", "Apple Braeburn 1", "Apple Core 1",
            "Apple Crimson Snow 1", "Apple Golden 1", "Apple Golden 2", "Apple Golden 3", "Apple Granny Smith 1",
            "Apple Pink Lady 1", "Apple Red 1", "Apple Red 2", "Apple Red 3", "Apple Red Delicious 1",
            "Apple Red Yellow 1", "Apple Red Yellow 2", "Apple Rotten 1", "Apple hit 1", "Apple worm 1",
            "Apricot 1", "Avocado 1", "Avocado Black 1", "Avocado Green 1", "Avocado ripe 1", "Banana 1",
            "Banana 3", "Banana 4", "Banana Lady Finger 1", "Banana Red 1", "Beans 1", "Beetroot 1",
            "Blackberrie 1", "Blackberrie 2", "Blackberrie half rippen 1", "Blackberrie not rippen 1",
            "Blueberry 1", "Cabbage red 1", "Cabbage white 1", "Cactus fruit 1", "Cactus fruit green 1",
            "Cactus fruit red 1", "Caju seed 1", "Cantaloupe 1", "Cantaloupe 2", "Carambula 1", "Carrot 1",
            "Cauliflower 1", "Cherimoya 1", "Cherry 1", "Cherry 2", "Cherry 3", "Cherry 4", "Cherry 5",
            "Cherry Rainier 1", "Cherry Rainier 2", "Cherry Rainier 3", "Cherry Sour 1", "Cherry Wax Black 1",
            "Cherry Wax Red 1", "Cherry Wax Red 2", "Cherry Wax Red 3", "Cherry Wax Yellow 1",
            "Cherry Wax not ripen 1", "Cherry Wax not ripen 2", "Chestnut 1", "Clementine 1", "Cocos 1",
            "Corn 1", "Corn Husk 1", "Cucumber 1", "Cucumber 10", "Cucumber 11", "Cucumber 3", "Cucumber 4",
            "Cucumber 5", "Cucumber 7", "Cucumber 9", "Cucumber Ripe 1", "Cucumber Ripe 2", "Dates 1",
            "Eggplant 1", "Eggplant long 1", "Fig 1", "Ginger Root 1", "Gooseberry 1", "Granadilla 1",
            "Grape Blue 1", "Grape Pink 1", "Grape White 1", "Grape White 2", "Grape White 3", "Grape White 4",
            "Grapefruit Pink 1", "Grapefruit White 1", "Guava 1", "Hazelnut 1", "Huckleberry 1", "Kaki 1",
            "Kiwi 1", "Kohlrabi 1", "Kumquats 1", "Lemon 1", "Lemon Meyer 1", "Limes 1", "Lychee 1",
            "Mandarine 1", "Mango 1", "Mango Red 1", "Mangostan 1", "Maracuja 1", "Melon Piel de Sapo 1",
            "Mulberry 1", "Nectarine 1", "Nectarine Flat 1", "Nut Forest 1", "Nut Pecan 1", "Onion Red 1",
            "Onion Red Peeled 1", "Onion White 1", "Orange 1", "Papaya 1", "Passion Fruit 1", "Peach 1",
            "Peach 2", "Peach Flat 1", "Pear 1", "Pear 2", "Pear 3", "Pear Abate 1", "Pear Forelle 1",
            "Pear Kaiser 1", "Pear Monster 1", "Pear Red 1", "Pear Stone 1", "Pear Williams 1", "Pepino 1",
            "Pepper Green 1", "Pepper Orange 1", "Pepper Red 1", "Pepper Yellow 1", "Physalis 1",
            "Physalis with Husk 1", "Pineapple 1", "Pineapple Mini 1", "Pistachio 1", "Pitahaya Red 1",
            "Plum 1", "Plum 2", "Plum 3", "Pomegranate 1", "Pomelo Sweetie 1", "Potato Red 1",
            "Potato Red Washed 1", "Potato Sweet 1", "Potato White 1", "Quince 1", "Quince 2", "Quince 3",
            "Quince 4", "Rambutan 1", "Raspberry 1", "Redcurrant 1", "Salak 1", "Strawberry 1",
            "Strawberry Wedge 1", "Tamarillo 1", "Tangelo 1", "Tomato 1", "Tomato 10", "Tomato 2", "Tomato 3",
            "Tomato 4", "Tomato 5", "Tomato 7", "Tomato 8", "Tomato 9", "Tomato Cherry Maroon 1",
            "Tomato Cherry Orange 1", "Tomato Cherry Red 1", "Tomato Cherry Red 2", "Tomato Cherry Yellow 1",
            "Tomato Heart 1", "Tomato Maroon 1", "Tomato Maroon 2", "Tomato Yellow 1", "Tomato not Ripen 1",
            "Walnut 1", "Watermelon 1", "Zucchini 1", "Zucchini dark 1"
        ]
        # Verificar que el número de clases coincida con la salida del modelo
        if len(class_names) != output_data_cls.shape[1]:
            self.get_logger().warning(
                f"El número de clases en class_names ({len(class_names)}) "
                f"no coincide con la salida del modelo ({output_data_cls.shape[1]}). "
                "Asegúrate de que la lista class_names sea correcta."
            )
            # Usar placeholders si hay discrepancia para evitar IndexError
            predicted_fruit = f"Clase_{predicted_class_index}"
        elif predicted_class_index >= len(class_names):
            self.get_logger().error(
                f"predicted_class_index ({predicted_class_index}) está fuera de rango para class_names (tamaño {len(class_names)}). "
                "Esto no debería ocurrir si las longitudes coinciden y argmax funciona como se espera."
            )
            predicted_fruit = f"Error_Clase_{predicted_class_index}"
        else:
            predicted_fruit = class_names[predicted_class_index]
        
        self.get_logger().info(f"🍓 DETECCIÓN: {predicted_fruit} con confianza: {confidence_score:.2f}")
        
        # Mostrar las 3 mejores detecciones
        self.get_logger().info("🔝 TOP 3 DETECCIONES:")
        for i, (idx, score) in enumerate(zip(top_indices, top_scores)):
            if idx < len(class_names):
                fruit_name = class_names[idx]
            else:
                fruit_name = f"Clase_{idx}"
            self.get_logger().info(f"   {i+1}. {fruit_name}: {score:.2f}")

        if confidence_score >= CONFIDENCE_THRESHOLD:
            # Publicar el resultado
            # Si es clasificación, podrías publicar el nombre de la fruta y su confianza.
            detection_msg = String()
            detection_msg.data = f"Fruta: {predicted_fruit}, Confianza: {confidence_score:.2f}"
            self.detection_publisher.publish(detection_msg)
            self.get_logger().info(f'✅ Publicando: {detection_msg.data}')
        else:
            self.get_logger().info(f'❌ Clasificación por debajo del umbral de confianza ({CONFIDENCE_THRESHOLD}).')


        # SI FUERA UN MODELO DE DETECCIÓN (YOLO para bounding boxes):
        # El post-procesamiento sería más complejo. Necesitarías interpretar
        # los bounding boxes, scores y clases de la salida del modelo.
        # output_data_detection = self.interpreter.get_tensor(self.output_details[0]['index'])
        # Por ejemplo, para un modelo YOLOv8 TFLite de detección, output_details podría tener
        # una forma como [1, num_detections, 4+num_classes] donde 4 son (x,y,w,h) o (x1,y1,x2,y2)
        # y num_classes son los scores por cada clase.
        # Deberías aplicar Non-Max Suppression (NMS) y filtrar por confianza.

        # Ejemplo de cómo se vería el bucle para detección (MUY SIMPLIFICADO):
        # for detection in output_data_detection[0]: # Iterar sobre las detecciones
        #     # Extraer bounding box, score de objeto, y scores de clase
        #     box = detection[:4]
        #     obj_score = detection[4] # Esto varía según el formato exacto de salida del modelo
        #     class_scores = detection[5:] # Esto varía
        #     class_id = np.argmax(class_scores)
        #     confidence = obj_score * class_scores[class_id] # O solo obj_score dependiendo del modelo
            
        #     if confidence > CONFIDENCE_THRESHOLD:
        #         # Crear mensaje de detección (idealmente un mensaje custom)
        #         # Dibujar en cv_image (opcional)
        #         # Publicar
        #         self.get_logger().info(f"Detectado objeto de clase {class_id} con confianza {confidence}")
        #         # detection_msg = String() # Reemplazar con mensaje customizado
        #         # detection_msg.data = f"Objeto: Clase_{class_id}, Confianza: {confidence:.2f}, Box: {box}"
        #         # self.detection_publisher.publish(detection_msg)
        #         pass # Fin del ejemplo de detección


def main(args=None):
    """Función principal para ejecutar el nodo de detección de frutas."""
    rclpy.init(args=args)
    fruit_detector_node = FruitDetectorNode()
    try:
        rclpy.spin(fruit_detector_node)
    except KeyboardInterrupt:
        fruit_detector_node.get_logger().info('Cerrando nodo por KeyboardInterrupt.')
    finally:
        # Destruir el nodo explícitamente
        # (opcional - Done for good measure)
        fruit_detector_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 