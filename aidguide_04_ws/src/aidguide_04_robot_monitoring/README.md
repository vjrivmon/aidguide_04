# Monitorización y Diagnóstico del Robot (aidguide_04_robot_monitoring)

Este paquete proporciona un sistema completo para monitorizar y diagnosticar el estado del robot, incluyendo el estado de batería, la salud del hardware, la temperatura y los mensajes de log.

## Características

- Monitorización de batería en tiempo real: nivel, estado de carga y salud.
- Diagnóstico de hardware: estado de motores, sensores, CPU y memoria.
- Monitorización de temperatura: alerta de sobrecalentamiento.
- Sistema de registro de mensajes: debug, información, advertencias y errores.
- Panel de control centralizado para visualizar toda la información.

## Tópicos

El paquete publica y se suscribe a los siguientes tópicos:

- `/battery_status` (sensor_msgs/BatteryState): Información sobre el nivel y estado de la batería.
- `/hardware_health` (diagnostic_msgs/DiagnosticArray): Diagnóstico de componentes del robot.
- `/temperature_sensor` (sensor_msgs/Temperature): Temperatura del sistema.
- `/log_messages` (rosgraph_msgs/Log): Mensajes de depuración y errores.

## Nodos

- `battery_monitor`: Simula y publica el estado de la batería.
- `hardware_monitor`: Monitoriza y diagnostica el estado de los componentes del hardware.
- `temperature_monitor`: Simula y publica la temperatura del sistema.
- `log_monitor`: Genera y publica mensajes de log con diferentes niveles de severidad.
- `monitoring_dashboard`: Panel central que recopila todas las métricas y muestra un resumen.

## Uso

Para iniciar todo el sistema de monitorización junto con el mundo de Gazebo:

```
ros2 launch aidguide_04_robot_monitoring monitoring.launch.py
```

## Dependencias

- `rclpy`: Cliente Python para ROS2
- `sensor_msgs`: Para mensajes de batería y temperatura
- `diagnostic_msgs`: Para mensajes de diagnóstico
- `rosgraph_msgs`: Para mensajes de log
- `aidguide_04_world`: Para cargar el mundo de simulación 