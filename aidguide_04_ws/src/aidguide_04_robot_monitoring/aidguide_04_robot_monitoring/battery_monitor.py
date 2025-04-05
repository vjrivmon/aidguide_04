import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import random
import time

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.publisher = self.create_publisher(BatteryState, '/battery_status', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_status)
        self.get_logger().info('Nodo monitor de batería iniciado')
        
        # Valores iniciales de la batería
        self.current_charge = 100.0
        self.discharge_rate = 0.02  # Tasa de descarga por segundo
        self.is_charging = False
        self.last_update = time.time()

    def publish_battery_status(self):
        # Actualizar el estado de la batería
        current_time = time.time()
        elapsed_time = current_time - self.last_update
        self.last_update = current_time
        
        # Simular carga/descarga
        if self.is_charging:
            self.current_charge += 0.1 * elapsed_time
            if self.current_charge >= 100.0:
                self.current_charge = 100.0
                # 10% de probabilidad de detener la carga
                if random.random() < 0.1:
                    self.is_charging = False
        else:
            self.current_charge -= self.discharge_rate * elapsed_time
            if self.current_charge <= 20.0:
                # 20% de probabilidad de comenzar a cargar cuando está baja
                if random.random() < 0.2:
                    self.is_charging = True
            
            if self.current_charge <= 0.0:
                self.current_charge = 0.0
                self.is_charging = True  # Iniciar carga de emergencia
        
        # Crear y publicar el mensaje
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = float(self.current_charge)
        msg.voltage = 12.0 + (self.current_charge / 100.0) * 2.0  # Simular voltaje entre 12-14V
        msg.current = 2.0 if self.is_charging else -1.0  # Positivo cuando carga, negativo cuando descarga
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING if self.is_charging 
            else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        
        # Establecer nivel de alerta basado en el porcentaje de batería
        if self.current_charge < 10.0:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif self.current_charge < 20.0:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERHEAT
        else:
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            
        self.publisher.publish(msg)
        self.get_logger().info(f'Estado de batería: {self.current_charge:.1f}%, {"Cargando" if self.is_charging else "Descargando"}')


def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 