#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time
import datetime

class LogMonitor(Node):
    def __init__(self):
        super().__init__('log_monitor')
        self.publisher = self.create_publisher(String, '/log_messages', 10)
        self.timer = self.create_timer(5.0, self.publish_log)
        self.get_logger().info('Nodo monitor de logs iniciado')
        
        # Definir categorías de mensajes
        self.info_messages = [
            "Navegación funcionando correctamente",
            "Conexión con estación base estable",
            "Actualización de firmware completada",
            "Batería en niveles óptimos",
            "Sensores calibrados correctamente",
            "Inicio de ruta de patrulla",
            "Cambio a modo autónomo",
            "Velocidad de crucero establecida",
            "Transferencia de datos completada",
            "Comunicación con otros robots establecida"
        ]
        self.warning_messages = [
            "Obstáculo detectado, redirigiendo ruta",
            "Conexión WiFi intermitente",
            "Batería por debajo del 30%",
            "Temperatura elevada, reduciendo carga de trabajo",
            "Precisión GPS reducida",
            "Humedad ambiental elevada",
            "Nivel de ruido en sensores aumentando",
            "Retraso en comunicaciones detectado",
            "Área desconocida, usando mapa local",
            "Inicio de modo ahorro de energía"
        ]
        self.error_messages = [
            "Error en sistema de navegación",
            "Conexión perdida con estación base",
            "Batería crítica, iniciando apagado",
            "Sobrecalentamiento detectado en motor izquierdo",
            "Sensor de distancia frontal no responde",
            "Error en unidad de procesamiento",
            "Pérdida de datos de odometría",
            "Fallo en comunicación interna",
            "Error al acceder al mapa global",
            "Detección de colisión"
        ]
        
        # Contador para el número de mensaje
        self.message_count = 0

    def publish_log(self):
        # Incrementar contador
        self.message_count += 1
        
        # Determinar tipo de mensaje basado en probabilidad
        rand_val = random.random()
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        if rand_val < 0.7:  # 70% mensajes informativos
            category = "INFO"
            message = random.choice(self.info_messages)
            self.get_logger().info(f"Log generado: {message}")
        elif rand_val < 0.9:  # 20% advertencias
            category = "WARN"
            message = random.choice(self.warning_messages)
            self.get_logger().warn(f"Log generado: {message}")
        else:  # 10% errores
            category = "ERROR"
            message = random.choice(self.error_messages)
            self.get_logger().error(f"Log generado: {message}")
        
        # Crear y publicar el mensaje
        log_msg = String()
        log_msg.data = f"[{timestamp}] [{category}] [Msg #{self.message_count}]: {message}"
        
        self.publisher.publish(log_msg)


def main(args=None):
    rclpy.init(args=args)
    log_monitor = LogMonitor()
    rclpy.spin(log_monitor)
    log_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 