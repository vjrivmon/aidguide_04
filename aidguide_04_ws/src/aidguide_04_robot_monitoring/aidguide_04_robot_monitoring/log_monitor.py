import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Log
import random
import time

class LogMonitor(Node):
    def __init__(self):
        super().__init__('log_monitor')
        self.publisher = self.create_publisher(Log, '/log_messages', 10)
        self.timer = self.create_timer(3.0, self.publish_log_messages)
        self.get_logger().info('Nodo monitor de logs iniciado')
        
        # Mensajes de ejemplo para cada nivel de log
        self.debug_messages = [
            "Inicializando módulo de navegación",
            "Cargando parámetros de configuración",
            "Conectando con sensores externos",
            "Comprobando disponibilidad de servicios",
            "Estado de memoria: OK",
            "Calibrando sensores de distancia",
            "Verificando conexiones a la red"
        ]
        
        self.info_messages = [
            "Robot en modo de navegación autónoma",
            "Mapa del entorno cargado correctamente",
            "Batería al 85%, funcionamiento normal",
            "Velocidad de desplazamiento: 0.5 m/s",
            "Objetivo de navegación alcanzado",
            "Nuevo objetivo recibido: Posición (x=2.5, y=3.2)",
            "Comunicación con estación base establecida"
        ]
        
        self.warn_messages = [
            "Obstáculo detectado, ajustando ruta",
            "Nivel de batería por debajo del 30%",
            "Temperatura del motor ligeramente elevada",
            "Conexión intermitente con estación base",
            "Precisión de localización reducida",
            "Sensor de proximidad con interferencias",
            "Memoria del sistema por encima del 80% de uso"
        ]
        
        self.error_messages = [
            "Fallo en sensor de distancia frontal",
            "Error de comunicación con controlador del motor",
            "No se puede acceder al mapa del entorno",
            "Timeout en respuesta del servicio de navegación",
            "Colisión detectada, deteniendo movimiento",
            "Temperatura crítica en componente principal",
            "Error en el sistema de visión"
        ]
        
        self.fatal_messages = [
            "Sistema de navegación no responde",
            "Pérdida de comunicación con todos los controladores",
            "Fallo crítico en sistema de alimentación",
            "Error irrecuperable en software principal",
            "Apagado de emergencia activado",
            "Fallo en sistema de seguridad principal",
            "Pérdida de control del robot"
        ]
        
        # Probabilidades de ocurrencia para cada nivel
        self.level_probabilities = {
            Log.DEBUG: 0.4,    # 40% para mensajes de depuración
            Log.INFO: 0.3,     # 30% para mensajes informativos
            Log.WARN: 0.15,    # 15% para advertencias
            Log.ERROR: 0.1,    # 10% para errores
            Log.FATAL: 0.05    # 5% para errores fatales
        }
        
        # Contadores para registrar estadísticas
        self.log_counts = {
            Log.DEBUG: 0,
            Log.INFO: 0,
            Log.WARN: 0,
            Log.ERROR: 0,
            Log.FATAL: 0
        }

    def publish_log_messages(self):
        # Determinar el nivel del mensaje
        level = self.select_random_level()
        
        # Seleccionar un mensaje aleatorio según el nivel
        if level == Log.DEBUG:
            message = random.choice(self.debug_messages)
            messages_list = self.debug_messages
        elif level == Log.INFO:
            message = random.choice(self.info_messages)
            messages_list = self.info_messages
        elif level == Log.WARN:
            message = random.choice(self.warn_messages)
            messages_list = self.warn_messages
        elif level == Log.ERROR:
            message = random.choice(self.error_messages)
            messages_list = self.error_messages
        else:  # FATAL
            message = random.choice(self.fatal_messages)
            messages_list = self.fatal_messages
        
        # Crear y enviar el mensaje de log
        msg = Log()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.level = level
        msg.name = "/robot/sistema"
        msg.msg = message
        msg.file = "robot_system.py"
        msg.function = "process_" + ("error" if level >= Log.ERROR else "status")
        msg.line = random.randint(10, 500)
        
        self.publisher.publish(msg)
        
        # Actualizar contadores
        self.log_counts[level] += 1
        
        # Mostrar mensaje en el log del nodo con el nivel apropiado
        if level == Log.DEBUG:
            self.get_logger().debug(f"[DEBUG] {message}")
        elif level == Log.INFO:
            self.get_logger().info(f"[INFO] {message}")
        elif level == Log.WARN:
            self.get_logger().warn(f"[WARN] {message}")
        elif level == Log.ERROR:
            self.get_logger().error(f"[ERROR] {message}")
        else:  # FATAL
            self.get_logger().fatal(f"[FATAL] {message}")
        
        # Mostrar estadísticas cada 10 mensajes
        total_messages = sum(self.log_counts.values())
        if total_messages % 10 == 0:
            self.get_logger().info(
                f"Estadísticas de mensajes - "
                f"DEBUG: {self.log_counts[Log.DEBUG]}, "
                f"INFO: {self.log_counts[Log.INFO]}, "
                f"WARN: {self.log_counts[Log.WARN]}, "
                f"ERROR: {self.log_counts[Log.ERROR]}, "
                f"FATAL: {self.log_counts[Log.FATAL]}"
            )
    
    def select_random_level(self):
        """Selecciona un nivel de log basado en las probabilidades configuradas"""
        r = random.random()
        cumulative = 0
        for level, prob in self.level_probabilities.items():
            cumulative += prob
            if r <= cumulative:
                return level
        return Log.INFO  # Valor por defecto si algo sale mal


def main(args=None):
    rclpy.init(args=args)
    log_monitor = LogMonitor()
    rclpy.spin(log_monitor)
    log_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 