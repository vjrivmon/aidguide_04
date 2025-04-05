import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WeatherMonitor(Node):

    def __init__(self):
        super().__init__('weather_monitor')
        
        # Suscriptor para el topic /weather_info
        self.subscription = self.create_subscription(
            String,
            'weather_info',
            self.weather_callback,
            10)
        
        # Publisher para notificaciones
        self.publisher = self.create_publisher(
            String,
            'weather_alert',
            10)
        
        self.get_logger().info('WeatherMonitor node inicializado. Escuchando en /weather_info')

    def weather_callback(self, msg):
        # Procesar mensaje del clima
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


def main(args=None):
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