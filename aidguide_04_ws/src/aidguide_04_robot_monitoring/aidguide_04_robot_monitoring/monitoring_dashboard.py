import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState, Temperature
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
import time
import os
import datetime

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
            String,
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
        self.timer = self.create_timer(3.0, self.display_dashboard)
        
        # Informar inicio
        self.get_logger().info('Panel de Monitorización del Robot iniciado')
        self.print_header("PANEL DE MONITORIZACIÓN DEL ROBOT - INICIADO")
        self.get_logger().info("Esperando datos de los sensores y monitores...")

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
        if "ERROR" in msg.data:
            self.get_logger().error(f'Mensaje crítico: {msg.data}')

    def display_dashboard(self):
        """Muestra un resumen del estado del robot"""
        self.clear_terminal()
        
        # Cabecera con fecha y hora actual
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.print_header(f"PANEL DE MONITORIZACIÓN DEL ROBOT - {timestamp}")
        
        # RESUMEN DE ESTADO DEL SISTEMA
        self.print_section("RESUMEN DE ESTADO DEL SISTEMA")
        status_general = "OK"
        status_issues = []
        
        # Verificar cada subsistema
        if self.battery_status and self.battery_status.percentage < 20.0:
            status_general = "ADVERTENCIA"
            status_issues.append("Batería baja")
        
        if self.temperature and self.temperature.temperature > 50.0:
            status_general = "ADVERTENCIA"
            status_issues.append("Temperatura elevada")
        
        if self.hardware_status:
            warning_count = 0
            error_count = 0
            for status in self.hardware_status.status:
                if status.level == 1:
                    warning_count += 1
                elif status.level == 2:
                    error_count += 1
            
            if error_count > 0:
                status_general = "ERROR"
                status_issues.append(f"{error_count} componentes en estado de error")
            elif warning_count > 0:
                status_general = "ADVERTENCIA"
                status_issues.append(f"{warning_count} componentes con advertencias")
        
        # Colorear según estado
        if status_general == "OK":
            self.get_logger().info("✅ Estado general: OK - Todos los sistemas funcionando correctamente")
        elif status_general == "ADVERTENCIA":
            self.get_logger().warn(f"⚠️ Estado general: ADVERTENCIA - Problemas detectados: {', '.join(status_issues)}")
        else:
            self.get_logger().error(f"🚨 Estado general: ERROR - Problemas críticos: {', '.join(status_issues)}")
        
        # Información de batería
        self.print_section("ESTADO DE BATERÍA")
        if self.battery_status:
            percentage = self.battery_status.percentage
            status = "CARGANDO" if self.battery_status.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING else "DESCARGANDO"
            health = "BUENA" if self.battery_status.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_GOOD else "DETERIORADA"
            
            # Barra de progreso visual
            battery_bar = self.get_progress_bar(percentage, 40)
            
            if percentage < 10.0:
                self.get_logger().error(f"🔋 Nivel de batería: {percentage:.1f}% {battery_bar}")
            elif percentage < 20.0:
                self.get_logger().warn(f"🔋 Nivel de batería: {percentage:.1f}% {battery_bar}")
            else:
                self.get_logger().info(f"🔋 Nivel de batería: {percentage:.1f}% {battery_bar}")
                
            self.get_logger().info(f"⚡ Estado: {status}")
            self.get_logger().info(f"🔄 Salud: {health}")
            self.get_logger().info(f"⚡ Voltaje: {self.battery_status.voltage:.2f}V")
        else:
            self.get_logger().info("🔌 No hay datos de batería disponibles")
        
        # Información de temperatura
        self.print_section("TEMPERATURA")
        if self.temperature:
            temp = self.temperature.temperature
            
            # Indicador visual de temperatura
            if temp > 60.0:
                self.get_logger().error(f"🔥 Temperatura: {temp:.1f}°C - CRÍTICA")
            elif temp > 50.0:
                self.get_logger().warn(f"🔥 Temperatura: {temp:.1f}°C - ALTA")
            else:
                self.get_logger().info(f"🌡️ Temperatura: {temp:.1f}°C - NORMAL")
        else:
            self.get_logger().info("🌡️ No hay datos de temperatura disponibles")
        
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
            
            self.get_logger().info(f"✅ Componentes OK: {ok_count}")
            if warn_count > 0:
                self.get_logger().warn(f"⚠️ Componentes con Advertencias: {warn_count}")
            if error_count > 0:
                self.get_logger().error(f"🚨 Componentes con Errores: {error_count}")
            
            # Mostrar los componentes con problemas
            if warn_count + error_count > 0:
                self.get_logger().info("Componentes con problemas:")
                for status in self.hardware_status.status:
                    if status.level == 1:
                        self.get_logger().warn(f"  ⚠️ {status.name}: Advertencia")
                    elif status.level == 2:
                        self.get_logger().error(f"  🚨 {status.name}: ERROR")
        else:
            self.get_logger().info("🔧 No hay datos de hardware disponibles")
        
        # Mensajes de log recientes
        self.print_section("MENSAJES RECIENTES")
        if self.latest_logs:
            for msg in reversed(self.latest_logs[-5:]):  # Mostrar los 5 más recientes, orden inverso
                if "ERROR" in msg.data:
                    self.get_logger().error(f"🚨 {msg.data}")
                elif "WARN" in msg.data:
                    self.get_logger().warn(f"⚠️ {msg.data}")
                else:
                    self.get_logger().info(f"📝 {msg.data}")
        else:
            self.get_logger().info("📝 No hay mensajes recientes")
        
        # Pie de página
        self.get_logger().info("\n" + "-"*70)
        self.get_logger().info(f"Actualización: {timestamp} | Próxima actualización en 3 segundos")
        self.get_logger().info("-"*70)

    def clear_terminal(self):
        """Limpia la terminal para una mejor visualización (solo visual en el log)"""
        self.get_logger().info("\n" + "\n"*5)

    def print_header(self, text):
        """Imprime un encabezado destacado"""
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info(f"   {text}   ")
        self.get_logger().info("="*70 + "\n")

    def print_section(self, text):
        """Imprime un encabezado de sección"""
        self.get_logger().info("\n" + "-"*70)
        self.get_logger().info(f"--- {text} ---")
        self.get_logger().info("-"*70)
    
    def get_progress_bar(self, percentage, width=20):
        """Crea una barra de progreso visual"""
        filled_width = int(width * percentage / 100)
        bar = '█' * filled_width + '░' * (width - filled_width)
        return f"[{bar}]"


def main(args=None):
    rclpy.init(args=args)
    dashboard = MonitoringDashboard()
    rclpy.spin(dashboard)
    dashboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 