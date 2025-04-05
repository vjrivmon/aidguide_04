import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import random
import time

class HardwareMonitor(Node):
    def __init__(self):
        super().__init__('hardware_monitor')
        self.publisher = self.create_publisher(DiagnosticArray, '/hardware_health', 10)
        self.timer = self.create_timer(2.0, self.publish_hardware_status)
        self.get_logger().info('Nodo monitor de hardware iniciado')
        
        # Componentes del robot para monitorear
        self.components = {
            'Motor Frontal Izquierdo': {'status': DiagnosticStatus.OK, 'values': {'temperatura': 35.0, 'voltaje': 12.0, 'corriente': 1.0}},
            'Motor Frontal Derecho': {'status': DiagnosticStatus.OK, 'values': {'temperatura': 36.0, 'voltaje': 12.0, 'corriente': 1.1}},
            'Motor Trasero Izquierdo': {'status': DiagnosticStatus.OK, 'values': {'temperatura': 34.0, 'voltaje': 12.0, 'corriente': 1.0}},
            'Motor Trasero Derecho': {'status': DiagnosticStatus.OK, 'values': {'temperatura': 35.5, 'voltaje': 12.0, 'corriente': 1.05}},
            'Sensor de Distancia': {'status': DiagnosticStatus.OK, 'values': {'precisión': 98.5, 'error': 0.01}},
            'Cámara': {'status': DiagnosticStatus.OK, 'values': {'resolución': 720, 'fps': 30}},
            'CPU': {'status': DiagnosticStatus.OK, 'values': {'uso': 25.0, 'temperatura': 45.0, 'frecuencia': 1800}},
            'Memoria': {'status': DiagnosticStatus.OK, 'values': {'uso': 30.0, 'disponible': 70.0}}
        }
        
        self.last_update = time.time()

    def publish_hardware_status(self):
        # Actualizar estado de componentes (simulación)
        current_time = time.time()
        elapsed_time = current_time - self.last_update
        self.last_update = current_time
        
        # Crear mensaje de diagnóstico
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Simular cambios en los componentes
        self.update_components_status(elapsed_time)
        
        # Añadir todos los componentes al mensaje
        for name, data in self.components.items():
            status = DiagnosticStatus()
            status.name = name
            status.level = data['status']
            
            if data['status'] == DiagnosticStatus.OK:
                status.message = "Funcionamiento normal"
            elif data['status'] == DiagnosticStatus.WARN:
                status.message = "Advertencia - Verificar componente"
            else:  # ERROR
                status.message = "Error - Requiere mantenimiento"
                
            # Añadir los valores clave para cada componente
            for key, value in data['values'].items():
                kv = KeyValue()
                kv.key = key
                kv.value = str(value)
                status.values.append(kv)
                
            msg.status.append(status)
                
        self.publisher.publish(msg)
        
        # Informar sobre componentes con problemas
        problems = [name for name, data in self.components.items() if data['status'] != DiagnosticStatus.OK]
        if problems:
            self.get_logger().warn(f'Componentes con problemas: {", ".join(problems)}')
        else:
            self.get_logger().info('Todos los sistemas funcionando correctamente')

    def update_components_status(self, elapsed_time):
        # Simular cambios en los componentes
        for name, data in self.components.items():
            # Simular probabilidad de fallo (5% de probabilidad cada actualización)
            if random.random() < 0.05:
                # Provocar un cambio de estado
                if data['status'] == DiagnosticStatus.OK:
                    data['status'] = DiagnosticStatus.WARN
                elif data['status'] == DiagnosticStatus.WARN:
                    # 50% de probabilidad de recuperarse o empeorar
                    data['status'] = random.choice([DiagnosticStatus.OK, DiagnosticStatus.ERROR])
                else:  # ERROR
                    # 30% de probabilidad de recuperarse a advertencia
                    if random.random() < 0.3:
                        data['status'] = DiagnosticStatus.WARN
            
            # Simular cambios en los valores
            if name.startswith('Motor'):
                # Fluctuación de temperatura del motor
                data['values']['temperatura'] += random.uniform(-0.5, 1.0)
                # Ajustar el estado basado en la temperatura
                if data['values']['temperatura'] > 65.0:
                    data['status'] = DiagnosticStatus.ERROR
                elif data['values']['temperatura'] > 50.0:
                    data['status'] = DiagnosticStatus.WARN
            
            elif name == 'CPU':
                # Fluctuación en el uso de CPU
                data['values']['uso'] = max(10.0, min(100.0, data['values']['uso'] + random.uniform(-5.0, 5.0)))
                data['values']['temperatura'] = 40.0 + (data['values']['uso'] / 100.0) * 30.0
                
                # Ajustar estado basado en uso y temperatura
                if data['values']['temperatura'] > 75.0 or data['values']['uso'] > 90.0:
                    data['status'] = DiagnosticStatus.ERROR
                elif data['values']['temperatura'] > 65.0 or data['values']['uso'] > 80.0:
                    data['status'] = DiagnosticStatus.WARN
                else:
                    data['status'] = DiagnosticStatus.OK


def main(args=None):
    rclpy.init(args=args)
    hardware_monitor = HardwareMonitor()
    rclpy.spin(hardware_monitor)
    hardware_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 