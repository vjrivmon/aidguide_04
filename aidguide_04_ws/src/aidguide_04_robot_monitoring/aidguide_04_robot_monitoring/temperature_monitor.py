import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
import random
import time

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')
        self.publisher = self.create_publisher(Temperature, '/temperature_sensor', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)
        self.get_logger().info('Nodo monitor de temperatura iniciado')
        
        # Valores iniciales de temperatura
        self.current_temp = 35.0  # Temperatura inicial en grados Celsius
        self.ambient_temp = 25.0  # Temperatura ambiente
        self.target_temp = self.current_temp  # Temperatura objetivo para simulación
        self.last_update = time.time()
        
        # Configuración de umbrales de alerta
        self.warning_temp = 50.0
        self.critical_temp = 65.0

    def publish_temperature(self):
        # Actualizar la temperatura simulada
        current_time = time.time()
        elapsed_time = current_time - self.last_update
        self.last_update = current_time
        
        # Simular cambios en la temperatura basados en carga (simplificado)
        # Periódicamente cambiar la temperatura objetivo para simular cambios de carga
        if random.random() < 0.1:  # 10% de probabilidad de cambiar la tendencia
            # La temperatura objetivo fluctúa entre ambiente+5 y alerta-5
            self.target_temp = random.uniform(self.ambient_temp + 5.0, self.warning_temp - 5.0)
        
        # Temperatura actual se mueve hacia la temperatura objetivo
        temp_diff = self.target_temp - self.current_temp
        self.current_temp += temp_diff * 0.1 * elapsed_time
        
        # Añadir algo de ruido aleatorio
        self.current_temp += random.uniform(-0.2, 0.2)
        
        # Crear y publicar el mensaje
        msg = Temperature()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.temperature = float(self.current_temp)
        msg.variance = 0.05  # Varianza fija para la simulación
        
        self.publisher.publish(msg)
        
        # Registrar la información con niveles apropiados
        if self.current_temp >= self.critical_temp:
            self.get_logger().error(f'¡TEMPERATURA CRÍTICA! {self.current_temp:.1f}°C - ¡Se requiere apagado!')
        elif self.current_temp >= self.warning_temp:
            self.get_logger().warn(f'ALERTA de temperatura: {self.current_temp:.1f}°C - Reducir carga')
        else:
            self.get_logger().info(f'Temperatura: {self.current_temp:.1f}°C - Normal')


def main(args=None):
    rclpy.init(args=args)
    temperature_monitor = TemperatureMonitor()
    rclpy.spin(temperature_monitor)
    temperature_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 