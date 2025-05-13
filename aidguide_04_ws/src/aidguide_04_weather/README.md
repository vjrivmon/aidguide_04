# AidGuide Weather Monitor

Paquete de ROS2 para monitorear información del clima y generar alertas en caso de lluvia o calor extremo.

## Descripción

Este paquete se suscribe al tópico `/weather_info` que recibe mensajes del tipo `std_msgs/String` con información sobre el clima. El nodo analiza esta información y genera alertas si detecta:

- Lluvia
- Calor extremo (temperaturas altas)

Las alertas se publican en el tópico `/weather_alert` usando mensajes del tipo `std_msgs/String`.

## Instalación

1. Clonar el repositorio en el directorio `src` de tu workspace de ROS2
2. Compilar el paquete:
   ```
   colcon build --packages-select aidguide_04_weather
   ```
3. Sourcea el setup:
   ```
   source install/setup.bash
   ```

## Uso

Para iniciar el nodo de monitoreo del clima:

```
ros2 launch aidguide_04_weather weather_monitor_launch.py
```

Para enviar información de clima al nodo:

```
ros2 topic pub /weather_info std_msgs/String "data: 'Hoy hay lluvia intensa prevista en la zona'" --once
```

Para comprobar las alertas generadas:

```
ros2 topic echo /weather_alert
``` 