import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState, Temperature
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Log
import time
import os

class MonitoringDashboard(Node):
    def __init__(self):
        super().__init__('monitoring_dashboard')
        
        # QoS para suscripciones
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Suscribirse a los topics de monitorización
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_status',
            self.battery_callback,
            qos_profile
        )
        
        self.hardware_sub = self.create_subscription(
            DiagnosticArray,
            '/hardware_health',
            self.hardware_callback,
            qos_profile
        )
        
        self.temp_sub = self.create_subscription(
            Temperature,
            '/temperature_sensor',
            self.temperature_callback,
            qos_profile
        )
        
        self.log_sub = self.create_subscription(
            Log,
            '/log_messages',
            self.log_callback,
            qos_profile
        )
        
        # Variables de estado para almacenar la información más reciente
        self.battery_status = None
        self.hardware_status = None
        self.temperature = None
        self.latest_logs = []  # Lista de los últimos mensajes de log
        self.max_logs = 10     # Máximo número de mensajes a mantener
        
        # Temporizador para mostrar el resumen periódicamente
        self.timer = self.create_timer(5.0, self.display_dashboard)
        
        # Informar inicio
        self.get_logger().info('Panel de Monitorización del Robot iniciado')

    def battery_callback(self, msg):
        self.battery_status = msg
        
        # Emitir alertas específicas según el nivel de batería
        if msg.percentage < 10.0:
            self.get_logger().error(f'¡ALERTA CRÍTICA DE BATERÍA! Nivel: {msg.percentage:.1f}%')
        elif msg.percentage < 20.0:
            self.get_logger().warn(f'Batería baja: {msg.percentage:.1f}%')

    def hardware_callback(self, msg):
        self.hardware_status = msg
        
        # Verificar si hay componentes en estado de error
        for status in msg.status:
            if status.level == 2:  # ERROR
                self.get_logger().error(f'Error en componente: {status.name} - {status.message}')

    def temperature_callback(self, msg):
        self.temperature = msg
        
        # Alertar si la temperatura es muy alta
        if msg.temperature > 60.0:
            self.get_logger().error(f'¡TEMPERATURA CRÍTICA! {msg.temperature:.1f}°C')
        elif msg.temperature > 50.0:
            self.get_logger().warn(f'Temperatura elevada: {msg.temperature:.1f}°C')

    def log_callback(self, msg):
        # Añadir el mensaje a la lista de mensajes recientes
        self.latest_logs.append(msg)
        
        # Mantener solo los mensajes más recientes
        if len(self.latest_logs) > self.max_logs:
            self.latest_logs.pop(0)
        
        # Para mensajes críticos, mostrarlos inmediatamente
        if msg.level >= Log.ERROR:
            self.get_logger().error(f'Mensaje crítico: [{msg.name}] {msg.msg}')

    def display_dashboard(self):
        """Muestra un resumen del estado del robot"""
        self.clear_terminal()
        
        # Cabecera
        self.print_header("PANEL DE MONITORIZACIÓN DEL ROBOT")
        
        # Información de batería
        self.print_section("ESTADO DE BATERÍA")
        if self.battery_status:
            percentage = self.battery_status.percentage
            status = "CARGANDO" if self.battery_status.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING else "DESCARGANDO"
            health = "BUENA" if self.battery_status.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_GOOD else "DETERIORADA"
            
            self.get_logger().info(f"Nivel de batería: {percentage:.1f}%")
            self.get_logger().info(f"Estado: {status}")
            self.get_logger().info(f"Salud: {health}")
            self.get_logger().info(f"Voltaje: {self.battery_status.voltage:.2f}V")
        else:
            self.get_logger().info("No hay datos de batería disponibles")
        
        # Información de temperatura
        self.print_section("TEMPERATURA")
        if self.temperature:
            temp = self.temperature.temperature
            status = "CRÍTICA" if temp > 60.0 else "ALTA" if temp > 50.0 else "NORMAL"
            
            self.get_logger().info(f"Temperatura: {temp:.1f}°C")
            self.get_logger().info(f"Estado: {status}")
        else:
            self.get_logger().info("No hay datos de temperatura disponibles")
        
        # Salud del hardware
        self.print_section("SALUD DEL HARDWARE")
        if self.hardware_status:
            # Contar los componentes por estado
            ok_count = 0
            warn_count = 0
            error_count = 0
            
            for status in self.hardware_status.status:
                if status.level == 0:
                    ok_count += 1
                elif status.level == 1:
                    warn_count += 1
                else:
                    error_count += 1
            
            self.get_logger().info(f"Componentes OK: {ok_count}")
            if warn_count > 0:
                self.get_logger().warn(f"Componentes con Advertencias: {warn_count}")
            if error_count > 0:
                self.get_logger().error(f"Componentes con Errores: {error_count}")
            
            # Mostrar los componentes con problemas
            if warn_count + error_count > 0:
                self.get_logger().info("Componentes con problemas:")
                for status in self.hardware_status.status:
                    if status.level > 0:
                        level = "Advertencia" if status.level == 1 else "ERROR"
                        self.get_logger().info(f"  - {status.name}: {level}")
        else:
            self.get_logger().info("No hay datos de hardware disponibles")
        
        # Mensajes de log recientes
        self.print_section("MENSAJES RECIENTES")
        if self.latest_logs:
            for msg in reversed(self.latest_logs[-5:]):  # Mostrar los 5 más recientes, orden inverso
                level_str = {
                    Log.DEBUG: "DEBUG",
                    Log.INFO: "INFO",
                    Log.WARN: "WARN",
                    Log.ERROR: "ERROR",
                    Log.FATAL: "FATAL"
                }.get(msg.level, "UNKNOWN")
                
                self.get_logger().info(f"[{level_str}] {msg.msg}")
        else:
            self.get_logger().info("No hay mensajes recientes")

    def clear_terminal(self):
        """Limpia la terminal para una mejor visualización (solo visual en el log)"""
        self.get_logger().info("\n" + "="*50)

    def print_header(self, text):
        """Imprime un encabezado destacado"""
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info(f"   {text}   ")
        self.get_logger().info("="*50 + "\n")

    def print_section(self, text):
        """Imprime un encabezado de sección"""
        self.get_logger().info("\n" + "-"*50)
        self.get_logger().info(f"--- {text} ---")
        self.get_logger().info("-"*50)


def main(args=None):
    rclpy.init(args=args)
    dashboard = MonitoringDashboard()
    rclpy.spin(dashboard)
    dashboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 