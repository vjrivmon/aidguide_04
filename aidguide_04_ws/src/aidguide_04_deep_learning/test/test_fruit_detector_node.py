import unittest
from unittest.mock import patch, MagicMock, mock_open

import os
import numpy as np

# Necesitaremos simular rclpy y sus componentes si no se ejecuta en un entorno ROS completo
# Para pruebas unitarias puras, esto es preferible.
# Si 'rclpy' no está disponible en el entorno de prueba, estas líneas necesitarán más mocks.
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    # Suponemos que el módulo del nodo se puede importar.
    # Esto podría requerir ajustes en PYTHONPATH o en cómo se ejecutan los tests.
    from aidguide_04_deep_learning.fruit_detector_node import FruitDetectorNode, MODEL_BASENAME, PREFERRED_PRECISION, FALLBACK_PRECISION 
except ImportError:
    # Si rclpy no está disponible (ej. ejecución fuera de ROS), mockearlo.
    rclpy = MagicMock()
    Node = MagicMock
    Image = MagicMock()
    String = MagicMock()
    CvBridge = MagicMock()
    FruitDetectorNode = MagicMock() # Será reemplazado por el import real si está disponible
    MODEL_BASENAME = 'best'
    PREFERRED_PRECISION = 'float16'
    FALLBACK_PRECISION = 'float32'
    print("ADVERTENCIA: rclpy no encontrado, usando mocks. Asegúrate de que el entorno de test esté configurado correctamente si esto es inesperado.")


# Mockear tensorflow globalmente para no requerir la librería en el entorno de test
tf_mock = MagicMock()
# Simular la estructura tf.lite.Interpreter
interpreter_instance_mock = MagicMock()
tf_mock.lite.Interpreter.return_value = interpreter_instance_mock

# Lista de clases hardcodeada en el nodo original, la replicamos o usamos una versión corta para tests
DEFAULT_CLASS_NAMES = [
    "Apple 10", "Apple 11", "Banana 1", "Orange 1" # Ejemplo corto
]


class TestFruitDetectorNodeInit(unittest.TestCase):
    """Pruebas para la inicialización y carga del modelo en FruitDetectorNode."""

    def setUp(self):
        """Configuración inicial para cada test."""
        # Mockear rclpy.init para que no falle si no estamos en un entorno ROS
        patcher_rclpy_init = patch('rclpy.init')
        self.addCleanup(patcher_rclpy_init.stop)
        self.mock_rclpy_init = patcher_rclpy_init.start()

        # Mockear get_package_share_directory
        self.patcher_get_pkg_share = patch('aidguide_04_deep_learning.fruit_detector_node.get_package_share_directory')
        self.mock_get_pkg_share = self.patcher_get_pkg_share.start()
        self.mock_get_pkg_share.return_value = '/fake/share/aidguide_04_deep_learning'
        
        self.resource_path = '/fake/share/aidguide_04_deep_learning/resource'
        
        # Mockear tf.lite.Interpreter y os.path.exists
        # Estos mocks se aplican a nivel de clase o método según sea necesario
        self.patcher_tf_interpreter = patch('aidguide_04_deep_learning.fruit_detector_node.tf', tf_mock)
        self.mock_tf_lite = self.patcher_tf_interpreter.start()
        
        self.patcher_os_path_exists = patch('aidguide_04_deep_learning.fruit_detector_node.os.path.exists')
        self.mock_os_path_exists = self.patcher_os_path_exists.start()

    def tearDown(self):
        """Limpieza después de cada test."""
        self.patcher_get_pkg_share.stop()
        self.patcher_tf_interpreter.stop()
        self.patcher_os_path_exists.stop()
        interpreter_instance_mock.reset_mock() # Limpiar mocks del intérprete

    def _setup_model_existence(self, preferred_exists=False, fallback_exists=False):
        """Ayudante para configurar qué modelos simula os.path.exists."""
        preferred_model_path = os.path.join(self.resource_path, f"{MODEL_BASENAME}_{PREFERRED_PRECISION}.tflite")
        fallback_model_path = os.path.join(self.resource_path, f"{MODEL_BASENAME}_{FALLBACK_PRECISION}.tflite")

        def side_effect(path):
            if path == preferred_model_path:
                return preferred_exists
            if path == fallback_model_path:
                return fallback_exists
            return False # Para otros paths que podría verificar
        self.mock_os_path_exists.side_effect = side_effect

    @patch('aidguide_04_deep_learning.fruit_detector_node.Node.__init__') # Mock Node.__init__
    def test_init_loads_preferred_model(self, mock_node_init):
        """Test: El nodo carga el modelo preferido (float16) si existe."""
        self._setup_model_existence(preferred_exists=True, fallback_exists=True)
        
        # Simular detalles de entrada/salida del modelo
        interpreter_instance_mock.get_input_details.return_value = [{'shape': [1, 100, 100, 3], 'dtype': np.float32, 'index': 0}]
        interpreter_instance_mock.get_output_details.return_value = [{'shape': [1, len(DEFAULT_CLASS_NAMES)], 'index': 0}]

        node = FruitDetectorNode()

        mock_node_init.assert_called_once_with('fruit_detector_node')
        self.mock_get_pkg_share.assert_called_once_with('aidguide_04_deep_learning')
        
        expected_model_path = os.path.join(self.resource_path, f"{MODEL_BASENAME}_{PREFERRED_PRECISION}.tflite")
        self.mock_tf_lite.lite.Interpreter.assert_called_once_with(model_path=expected_model_path)
        interpreter_instance_mock.allocate_tensors.assert_called_once()
        self.assertIsNotNone(node.interpreter)
        self.assertIsNotNone(node.image_subscription)
        self.assertIsNotNone(node.detection_publisher)

    @patch('aidguide_04_deep_learning.fruit_detector_node.Node.__init__')
    def test_init_loads_fallback_model(self, mock_node_init):
        """Test: El nodo carga el modelo fallback (float32) si el preferido no existe."""
        self._setup_model_existence(preferred_exists=False, fallback_exists=True)
        interpreter_instance_mock.get_input_details.return_value = [{'shape': [1, 100, 100, 3], 'dtype': np.float32, 'index': 0}]
        interpreter_instance_mock.get_output_details.return_value = [{'shape': [1, len(DEFAULT_CLASS_NAMES)], 'index': 0}]

        node = FruitDetectorNode()

        mock_node_init.assert_called_once_with('fruit_detector_node')
        expected_model_path = os.path.join(self.resource_path, f"{MODEL_BASENAME}_{FALLBACK_PRECISION}.tflite")
        self.mock_tf_lite.lite.Interpreter.assert_called_once_with(model_path=expected_model_path)
        interpreter_instance_mock.allocate_tensors.assert_called_once()
        self.assertIsNotNone(node.interpreter)

    @patch('aidguide_04_deep_learning.fruit_detector_node.Node.__init__')
    def test_init_no_model_found(self, mock_node_init):
        """Test: El nodo maneja el caso donde ningún modelo TFLite es encontrado."""
        self._setup_model_existence(preferred_exists=False, fallback_exists=False)
        
        # Mock logger para verificar mensajes de error
        mock_logger = MagicMock()
        with patch.object(FruitDetectorNode, 'get_logger', return_value=mock_logger):
            node = FruitDetectorNode() # Se llama a __init__

        mock_node_init.assert_called_once_with('fruit_detector_node')
        self.mock_tf_lite.lite.Interpreter.assert_not_called() # No se debe intentar cargar
        
        # Verificar que se logueó el error de que no se encontró el modelo
        # La comprobación exacta del mensaje puede ser frágil, pero verificamos que se llamó a error.
        self.assertTrue(any("No se pudo encontrar ningún modelo TFLite" in call_args[0][0] for call_args in mock_logger.error.call_args_list))
        
        # Comprobar que el intérprete no se ha establecido y que el nodo podría estar en un estado no funcional
        self.assertFalse(hasattr(node, 'interpreter') or node.interpreter is not None)


    @patch('aidguide_04_deep_learning.fruit_detector_node.Node.__init__')
    def test_init_model_load_exception(self, mock_node_init):
        """Test: El nodo maneja excepciones durante la carga del modelo TFLite."""
        self._setup_model_existence(preferred_exists=True) # Simula que el archivo existe
        self.mock_tf_lite.lite.Interpreter.side_effect = Exception("Error de carga TFLite")

        mock_logger = MagicMock()
        with patch.object(FruitDetectorNode, 'get_logger', return_value=mock_logger):
            node = FruitDetectorNode()

        mock_node_init.assert_called_once_with('fruit_detector_node')
        self.mock_tf_lite.lite.Interpreter.assert_called_once() # Se intentó cargar
        self.assertTrue(any("Error al cargar el modelo TFLite" in call_args[0][0] for call_args in mock_logger.error.call_args_list))
        self.assertFalse(hasattr(node, 'interpreter') or node.interpreter is not None)


class TestFruitDetectorNodeProcessing(unittest.TestCase):
    """Pruebas para el procesamiento de imágenes y la lógica de callback."""

    @patch('aidguide_04_deep_learning.fruit_detector_node.Node.__init__')
    @patch('aidguide_04_deep_learning.fruit_detector_node.get_package_share_directory')
    @patch('aidguide_04_deep_learning.fruit_detector_node.os.path.exists')
    @patch('aidguide_04_deep_learning.fruit_detector_node.tf', new_callable=MagicMock) # Usar el tf_mock configurado globalmente o uno nuevo
    def setUp(self, mock_tf_global, mock_os_exists, mock_get_pkg_share, mock_node_init):
        """Configura una instancia del nodo con mocks para cada test de procesamiento."""
        # Configuración básica para que __init__ no falle
        mock_node_init.return_value = None
        mock_get_pkg_share.return_value = '/fake/share/aidguide_04_deep_learning'
        
        # Simular que el modelo preferido existe
        resource_path = '/fake/share/aidguide_04_deep_learning/resource'
        preferred_model_path = os.path.join(resource_path, f"{MODEL_BASENAME}_{PREFERRED_PRECISION}.tflite")
        mock_os_exists.side_effect = lambda path: path == preferred_model_path

        # Mockear el intérprete de TensorFlow Lite (reutilizar el global o el mock_tf_global)
        self.interpreter_mock = MagicMock() # Es el mismo que interpreter_instance_mock global, pero local para claridad
        mock_tf_global.lite.Interpreter.return_value = self.interpreter_mock
        
        # Simular detalles de entrada/salida del modelo (ajusta según el modelo real)
        self.mock_input_details = [{'shape': [1, 100, 100, 3], 'dtype': np.float32, 'index': 0}]
        self.mock_output_details = [{'shape': [1, len(DEFAULT_CLASS_NAMES)], 'index': 0}] # Usar DEFAULT_CLASS_NAMES del test
        self.interpreter_mock.get_input_details.return_value = self.mock_input_details
        self.interpreter_mock.get_output_details.return_value = self.mock_output_details
        
        # Mock rclpy.init (ya debería estar mockeado si ejecutas todos los tests, pero por si acaso)
        patcher_rclpy_init = patch('rclpy.init', MagicMock())
        self.addCleanup(patcher_rclpy_init.stop)
        patcher_rclpy_init.start()

        # Crear la instancia del nodo bajo prueba
        self.node = FruitDetectorNode()
        # Sobrescribir class_names con nuestra lista corta para los tests
        self.node.class_names = DEFAULT_CLASS_NAMES
        self.node.interpreter = self.interpreter_mock # Asegurarse de que el nodo usa nuestro mock
        self.node.input_details = self.mock_input_details
        self.node.output_details = self.mock_output_details

        # Mockear CvBridge y el publicador
        self.node.bridge = MagicMock()
        self.node.detection_publisher = MagicMock()
        self.node.get_logger = MagicMock(return_value=MagicMock()) # Mockear el logger también

    def test_preprocess_image_float32(self):
        """Test: _preprocess_image para entrada float32."""
        self.node.input_details[0]['dtype'] = np.float32
        mock_cv_image = np.random.randint(0, 256, (200, 200, 3), dtype=np.uint8)
        
        processed_data = self.node._preprocess_image(mock_cv_image)
        
        self.assertIsNotNone(processed_data)
        self.assertEqual(processed_data.shape, (1, 100, 100, 3)) # Segun input_details
        self.assertEqual(processed_data.dtype, np.float32)
        # Aquí podrías añadir una verificación de normalización si la activas en _preprocess_image
        # Por ejemplo, self.assertTrue(np.max(processed_data) <= 1.0)

    def test_preprocess_image_uint8(self):
        """Test: _preprocess_image para entrada uint8."""
        self.node.input_details[0]['dtype'] = np.uint8
        mock_cv_image = np.random.randint(0, 256, (200, 200, 3), dtype=np.uint8)

        processed_data = self.node._preprocess_image(mock_cv_image)

        self.assertIsNotNone(processed_data)
        self.assertEqual(processed_data.shape, (1, 100, 100, 3))
        self.assertEqual(processed_data.dtype, np.uint8) # Debería mantenerse como uint8

    def test_get_classification_from_output(self):
        """Test: _get_classification_from_output interpreta correctamente la salida del modelo."""
        # Simular una salida del modelo (scores para cada clase)
        # Ejemplo: Banana es la clase con mayor score (índice 2 en DEFAULT_CLASS_NAMES)
        mock_output_tensor = np.array([[0.1, 0.2, 0.9, 0.3]], dtype=np.float32) # Scores para 4 clases
        self.node.output_details[0]['shape'] = [1, len(DEFAULT_CLASS_NAMES)] # Asegurar consistencia

        fruit_name, confidence, top_3 = self.node._get_classification_from_output(mock_output_tensor)

        self.assertEqual(fruit_name, "Banana 1")
        self.assertAlmostEqual(confidence, 0.9, places=5)
        self.assertEqual(len(top_3), 3)
        self.assertEqual(top_3[0][0], "Banana 1")
        self.assertAlmostEqual(top_3[0][1], 0.9, places=5)
        self.assertEqual(top_3[1][0], "Orange 1") # Siguiente más alto
        self.assertAlmostEqual(top_3[1][1], 0.3, places=5)

    def test_get_classification_from_output_mismatch_classes(self):
        """Test: Manejo de discrepancia entre class_names y salida del modelo."""
        mock_output_tensor = np.array([[0.1, 0.8, 0.3]], dtype=np.float32) # Modelo con 3 clases
        self.node.class_names = ["Apple", "Banana"] # Pero solo 2 nombres de clase definidos
        self.node.output_details[0]['shape'] = [1, 3] # Salida del modelo tiene 3

        fruit_name, confidence, top_3 = self.node._get_classification_from_output(mock_output_tensor)
        
        # El más alto es el índice 1 (Banana)
        self.assertEqual(fruit_name, "Banana")
        self.assertAlmostEqual(confidence, 0.8, places=5)
        # El top 3 aún debería funcionar pero con nombres genéricos para la clase no mapeada
        self.assertEqual(top_3[0][0], "Banana")
        self.assertEqual(top_3[2][0], "ClaseDesconocida_0") # El de score 0.1 (índice 0)
        self.node.get_logger().warning.assert_called() # Verificar que se logueó la advertencia

    def test_image_callback_successful_detection(self):
        """Test: image_callback procesa una imagen y publica una detección."""
        mock_ros_image_msg = Image() # Crear un objeto Image mockeado
        mock_cv_image = np.random.randint(0, 256, (100, 100, 3), dtype=np.uint8)
        self.node.bridge.imgmsg_to_cv2.return_value = mock_cv_image

        # Simular preprocesamiento
        processed_image_mock = np.random.rand(1, 100, 100, 3).astype(np.float32)
        # Patch el método _preprocess_image de la instancia del nodo
        with patch.object(self.node, '_preprocess_image', return_value=processed_image_mock) as mock_preprocess:
            # Simular salida del modelo
            # Banana (índice 2) tiene el mayor score
            mock_model_output = np.array([[0.1, 0.2, 0.8, 0.1]], dtype=np.float32)
            self.interpreter_mock.get_tensor.return_value = mock_model_output

            # Llamar al callback
            self.node.image_callback(mock_ros_image_msg)

            # Verificaciones
            self.node.bridge.imgmsg_to_cv2.assert_called_once_with(mock_ros_image_msg, desired_encoding='bgr8')
            mock_preprocess.assert_called_once_with(mock_cv_image)
            self.interpreter_mock.set_tensor.assert_called_once_with(self.mock_input_details[0]['index'], processed_image_mock)
            self.interpreter_mock.invoke.assert_called_once()
            self.interpreter_mock.get_tensor.assert_called_once_with(self.mock_output_details[0]['index'])
            
            # Verificar que se publicó el mensaje correcto
            # (asumiendo CONFIDENCE_THRESHOLD es <= 0.8)
            self.node.detection_publisher.publish.assert_called_once()
            published_msg = self.node.detection_publisher.publish.call_args[0][0]
            self.assertIsInstance(published_msg, String)
            self.assertIn("Fruta: Banana 1", published_msg.data)
            self.assertIn("Confianza: 0.80", published_msg.data)
            self.node.get_logger().info.assert_any_call('✅ Publicando: Fruta: Banana 1, Confianza: 0.80')

    def test_image_callback_low_confidence(self):
        """Test: image_callback no publica si la confianza es baja."""
        mock_ros_image_msg = Image()
        mock_cv_image = np.zeros((100,100,3), dtype=np.uint8)
        self.node.bridge.imgmsg_to_cv2.return_value = mock_cv_image
        
        processed_image_mock = np.zeros((1,100,100,3), dtype=np.float32)
        with patch.object(self.node, '_preprocess_image', return_value=processed_image_mock):
            # Simular salida con baja confianza
            low_confidence_output = np.array([[0.1, 0.2, 0.3, 0.05]], dtype=np.float32) # Max es 0.3
            self.interpreter_mock.get_tensor.return_value = low_confidence_output
            original_threshold = self.node.CONFIDENCE_THRESHOLD
            self.node.CONFIDENCE_THRESHOLD = 0.5 # Asegurar un umbral para el test

            self.node.image_callback(mock_ros_image_msg)

            self.node.CONFIDENCE_THRESHOLD = original_threshold # Restaurar
            self.node.detection_publisher.publish.assert_not_called()
            self.node.get_logger().info.assert_any_call(f'❌ Clasificación por debajo del umbral de confianza ({0.5}).')

    def test_image_callback_cv_bridge_exception(self):
        """Test: image_callback maneja excepciones de CvBridge."""
        mock_ros_image_msg = Image()
        self.node.bridge.imgmsg_to_cv2.side_effect = Exception("Error de CvBridge")

        self.node.image_callback(mock_ros_image_msg)

        self.node.detection_publisher.publish.assert_not_called()
        self.interpreter_mock.set_tensor.assert_not_called()
        self.node.get_logger().error.assert_any_call("Error al convertir imagen de ROS a OpenCV: Error de CvBridge")

    def test_image_callback_interpreter_not_ready(self):
        """Test: image_callback no procesa si el intérprete no está listo."""
        self.node.interpreter = None # Simular que el intérprete no se cargó
        mock_ros_image_msg = Image()

        self.node.image_callback(mock_ros_image_msg)

        self.node.bridge.imgmsg_to_cv2.assert_not_called()
        self.node.get_logger().warn.assert_any_call("Intérprete o detalles del modelo no inicializados. Saltando callback.")


if __name__ == '__main__':
    # Esto permite ejecutar los tests directamente con 'python test_fruit_detector_node.py'
    # Sin embargo, es mejor usar 'colcon test' o 'python -m unittest'
    unittest.main() 