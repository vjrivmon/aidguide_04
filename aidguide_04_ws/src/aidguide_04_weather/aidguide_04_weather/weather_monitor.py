import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
import time

class WeatherMonitor(Node):

    """Clase WeatherMonitor.
    
    Implementa funcionalidad para WeatherMonitor.
    """
    def __init__(self):
        """Función Constructor.
        """
        super().__init__('weather_monitor')
        
        # Suscriptor para el topic /weather_info
        self.subscription = self.create_subscription(
            String,
            'weather_info',
            self.weather_callback,
            10)
        
        # Publisher para notificaciones de alertas meteorológicas
        self.publisher = self.create_publisher(
            String,
            'weather_alert',
            10)
        
        # Lista de posibles fenómenos atmosféricos con sus colores
        self.weather_phenomena = [
            {"type": "lluvia", "message": "ALERTA: Lluvia intensa prevista", "color": "blue"},
            {"type": "calor", "message": "ALERTA: Ola de calor (38°C)", "color": "red"},
            {"type": "nieve", "message": "ALERTA: Posible nevada ligera", "color": "white"},
            {"type": "niebla", "message": "ALERTA: Niebla densa, precaución", "color": "gray"},
            {"type": "viento", "message": "ALERTA: Vientos fuertes (80 km/h)", "color": "orange"},
            {"type": "tormenta", "message": "ALERTA: Tormenta eléctrica", "color": "purple"},
            {"type": "granizo", "message": "ALERTA: Posibilidad de granizo", "color": "cyan"}
        ]
        
        # Crear un timer que genere alertas cada 3 segundos
        self.timer = self.create_timer(3.0, self.generate_weather_alert)
        
        self.get_logger().info('WeatherMonitor node inicializado. Publicando alertas cada 3 segundos')

    def weather_callback(self, msg):
        # Procesar mensaje del clima
        """Función Weather callback.
        
        Args:
            msg (Any): Descripción del parámetro.
        """
        weather_info = msg.data
        self.get_logger().info(f'Información del clima recibida: {weather_info}')
        
        # Analizar condiciones climáticas
        alert_message = None
        
        if 'lluvia' in weather_info.lower():
            alert_message = 'ALERTA: Posibilidad de lluvia. Considere llevar paraguas.'
        elif 'calor' in weather_info.lower() or 'temperatura alta' in weather_info.lower():
            if any(str(temp) in weather_info for temp in range(35, 50)):
                alert_message = 'ALERTA: Calor extremo detectado. Manténgase hidratado y evite exposición directa al sol.'
        
        # Publicar alerta si es necesario
        if alert_message:
            alert = String()
            alert.data = alert_message
            self.publisher.publish(alert)
            self.get_logger().warning(alert_message)

    def generate_weather_alert(self):
        # Seleccionar un fenómeno aleatorio
        """Función Generate weather alert.
        """
        phenomenon = random.choice(self.weather_phenomena)
        
        # Construir mensaje con información del fenómeno y color
        message = f"{phenomenon['message']}|{phenomenon['type']}|{phenomenon['color']}"
        
        # Crear y publicar el mensaje
        alert = String()
        alert.data = message
        self.publisher.publish(alert)
        
        # Registrar en el log
        self.get_logger().info(f'Alerta meteorológica publicada: {phenomenon["type"]} (color: {phenomenon["color"]})')


def main(args=None):
    """Función Main.
    
    Args:
        args (Any): Descripción del parámetro.
    """
    rclpy.init(args=args)
    weather_monitor = WeatherMonitor()
    
    try:
        rclpy.spin(weather_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        weather_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 