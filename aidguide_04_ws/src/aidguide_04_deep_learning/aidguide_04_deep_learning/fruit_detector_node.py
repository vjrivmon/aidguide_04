#!/usr/bin/env python3
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
                self.interpreter = None # Asegurarse de que interpreter es None
                self.input_details = None
                self.output_details = None
                self.bridge = CvBridge() # Podría ser necesario para otros métodos si se llama
                # Configurar publicadores/suscriptores igualmente para evitar errores de atributo no encontrado,
                # pero el nodo no será funcional.
                self._setup_ros_communication()
                return

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
            # Considerar lanzar una excepción o un mecanismo para detener la inicialización completa
            self.interpreter = None # Asegurarse de que interpreter es None
            self.input_details = None
            self.output_details = None
            self.bridge = CvBridge() # Podría ser necesario para otros métodos si se llama
            # Configurar publicadores/suscriptores igualmente para evitar errores de atributo no encontrado,
            # pero el nodo no será funcional.
            self._setup_ros_communication()
            return

        # Obtener detalles de entrada y salida del modelo
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.get_logger().info(f"Detalles de entrada del modelo: {self.input_details}")
        self.get_logger().info(f"Detalles de salida del modelo: {self.output_details}")

        # CvBridge para convertir entre imágenes de ROS y OpenCV
        self.bridge = CvBridge()
        
        # Definir class_names como atributo de instancia
        self.class_names = [
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

        self._setup_ros_communication()

    def _setup_ros_communication(self):
        """Configura los suscriptores y publicadores de ROS."""
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

    def _preprocess_image(self, cv_image: np.ndarray) -> np.ndarray:
        """Preprocesa la imagen CV2 para la entrada del modelo TFLite."""
        self.get_logger().debug("Iniciando preprocesamiento de imagen...")
        if self.input_details is None:
            self.get_logger().error("Detalles de entrada del modelo no disponibles. Saltando preprocesamiento.")
            return None # O lanzar excepción

        input_shape = self.input_details[0]['shape']
        img_height = input_shape[1]
        img_width = input_shape[2]

        input_image_resized = cv2.resize(cv_image, (img_width, img_height))
        
        # La conversión a RGB depende de cómo fue entrenado el modelo.
        # Si el modelo espera RGB pero OpenCV entrega BGR:
        # input_image_rgb = cv2.cvtColor(input_image_resized, cv2.COLOR_BGR2RGB)
        # Por ahora, asumimos que el modelo trabaja con BGR o ya está en el formato correcto.
        input_image_expanded = np.expand_dims(input_image_resized, axis=0)
        
        input_data = input_image_expanded
        if self.input_details[0]['dtype'] == np.float32:
            input_data = np.float32(input_image_expanded)
            # Normalización (0-1) si el modelo float32 lo espera.
            # Descomentar si es necesario:
            # input_data = input_data / 255.0 
            self.get_logger().debug(f"Imagen preprocesada a float32, forma: {input_data.shape}")
        elif self.input_details[0]['dtype'] == np.uint8:
            # Si el modelo espera uint8, input_image_expanded debería estar bien.
            self.get_logger().debug(f"Imagen preprocesada a uint8, forma: {input_data.shape}")
        
        return input_data

    def _get_classification_from_output(self, output_data_cls: np.ndarray) -> tuple[str, float, list[tuple[str, float]]]:
        """
        Interpreta la salida del modelo de clasificación.

        Args:
            output_data_cls: Array NumPy de la salida del tensor del modelo.

        Returns:
            Una tupla conteniendo:
                - nombre_clase_predicha (str)
                - confianza (float)
                - lista_top_3_predicciones (list[tuple[str, float]])
        """
        self.get_logger().debug("Procesando salida de clasificación...")
        predicted_class_index = int(np.argmax(output_data_cls[0]))
        confidence_score = float(output_data_cls[0][predicted_class_index])

        top_indices = np.argsort(output_data_cls[0])[-3:][::-1]
        # top_scores = output_data_cls[0][top_indices] # No se usa directamente, se accede por índice

        predicted_fruit_name = f"ClaseDesconocida_{predicted_class_index}"
        
        if not hasattr(self, 'class_names') or not self.class_names:
            self.get_logger().error("Lista 'class_names' no definida o vacía en el nodo.")
            # Manejar el caso donde class_names no está disponible
            top_3_predictions = [(f"ClaseDesconocida_{idx}", float(output_data_cls[0][idx])) for idx in top_indices]
            return predicted_fruit_name, confidence_score, top_3_predictions

        if len(self.class_names) != output_data_cls.shape[1]:
            self.get_logger().warning(
                f"El número de clases en self.class_names ({len(self.class_names)}) "
                f"no coincide con la salida del modelo ({output_data_cls.shape[1]}). "
                "Asegúrate de que la lista class_names sea correcta."
            )
            # Aún así intentar mapear lo que se pueda
        
        if 0 <= predicted_class_index < len(self.class_names):
            predicted_fruit_name = self.class_names[predicted_class_index]
        else:
            self.get_logger().error(
                f"predicted_class_index ({predicted_class_index}) está fuera de rango para self.class_names (tamaño {len(self.class_names)}). "
            )

        top_3_predictions = []
        for idx_val in top_indices: # Iterar sobre los valores de los índices
            idx = int(idx_val) # Asegurar que el índice es un entero
            score_val = float(output_data_cls[0][idx])
            fruit_name_top = f"ClaseDesconocida_{idx}"
            if 0 <= idx < len(self.class_names):
                fruit_name_top = self.class_names[idx]
            top_3_predictions.append((fruit_name_top, score_val))
            self.get_logger().debug(f"  Top: {fruit_name_top}: {score_val:.2f}")
            
        self.get_logger().debug(f"Clasificación final: {predicted_fruit_name}, Confianza: {confidence_score:.2f}")
        return predicted_fruit_name, confidence_score, top_3_predictions

    def image_callback(self, msg: Image):
        """Callback que se ejecuta cada vez que se recibe una nueva imagen.

        Args:
            msg (sensor_msgs.msg.Image): El mensaje de imagen recibido.
        """
        if not self.interpreter or not self.input_details or not self.output_details:
            self.get_logger().warn("Intérprete o detalles del modelo no inicializados. Saltando callback.")
            return

        self.get_logger().info(f'✨ Recibida imagen con timestamp: {msg.header.stamp}')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen de ROS a OpenCV: {e}")
            return

        input_data = self._preprocess_image(cv_image)
        if input_data is None:
            self.get_logger().error("Fallo en el preprocesamiento de la imagen.")
            return

        # Establecer el tensor de entrada
        try:
            self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        except Exception as e:
            self.get_logger().error(f"Error al establecer el tensor de entrada: {e}")
            return

        # Ejecutar la inferencia
        try:
            self.interpreter.invoke()
        except Exception as e:
            self.get_logger().error(f"Error durante la inferencia del modelo: {e}")
            return
        
        # Obtener los resultados de la inferencia
        try:
            output_data_cls = self.interpreter.get_tensor(self.output_details[0]['index'])
        except Exception as e:
            self.get_logger().error(f"Error al obtener el tensor de salida: {e}")
            return

        predicted_fruit, confidence, top_3 = self._get_classification_from_output(output_data_cls)
        
        self.get_logger().info(f"🍓 DETECCIÓN PRINCIPAL: {predicted_fruit} con confianza: {confidence:.2f}")
        self.get_logger().info("🔝 TOP 3 DETECCIONES:")
        for i, (fruit_name, score) in enumerate(top_3):
            self.get_logger().info(f"   {i+1}. {fruit_name}: {score:.2f}")

        if confidence >= CONFIDENCE_THRESHOLD:
            detection_msg = String()
            detection_msg.data = f"Fruta: {predicted_fruit}, Confianza: {confidence:.2f}"
            self.detection_publisher.publish(detection_msg)
            self.get_logger().info(f'✅ Publicando: {detection_msg.data}')
        else:
            self.get_logger().info(f'❌ Clasificación por debajo del umbral de confianza ({CONFIDENCE_THRESHOLD}).')

        # El código de ejemplo para detección de objetos (bounding boxes) se mantiene comentado
        # ya que el nodo actualmente implementa clasificación.


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