# AidGuide 04 SLAM

Este paquete proporciona funcionalidades para SLAM (Simultaneous Localization and Mapping) y localización para el robot AidGuide 04.

## Descripción

El paquete `aidguide_04_slam` ofrece las siguientes funcionalidades:

- **SLAM**: Utiliza slam_toolbox para crear mapas del entorno mientras el robot se mueve.
- **Localización**: Permite al robot ubicarse en un mapa previamente creado utilizando AMCL.
- **Gestión de mapas**: Facilita guardar y cargar mapas para su uso posterior.

## Requisitos Previos

- ROS 2 Galactic instalado
- Paquetes necesarios:
  ```
  sudo apt install ros-galactic-slam-toolbox ros-galactic-nav2-map-server ros-galactic-nav2-amcl ros-galactic-nav2-lifecycle-manager
  ```

## Uso

### 1. Crear un mapa (SLAM)

Para iniciar el proceso de creación de mapa:

```bash
ros2 launch aidguide_04_slam slam_launch.py
```

Opciones adicionales:
- `use_sim_time:=true` - Para usar tiempo de simulación (necesario con Gazebo)

Durante el proceso de SLAM, mueve el robot por el entorno para crear el mapa. Puedes utilizar herramientas como `teleop_twist_keyboard` para controlar el robot.

Para guardar el mapa generado:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/aidguide_maps/mi_mapa
```

### 2. Localización en un mapa existente

Para que el robot se localice dentro de un mapa previamente generado:

```bash
ros2 launch aidguide_04_slam localization_launch.py map_file:=/ruta/al/mapa.yaml
```

Opciones adicionales:
- `use_sim_time:=true` - Para usar tiempo de simulación
- `map_file:=ruta/al/mapa.yaml` - Para especificar un mapa diferente al predeterminado

### 3. Estado de localización

El nodo de localización publica el estado del robot en el topic `/localization/status` con los siguientes posibles estados:

- **ESPERANDO_MAPA**: El sistema aún no ha recibido el mapa.
- **LOCALIZANDO**: El robot está intentando determinar su posición en el mapa.
- **LOCALIZADO**: El robot ha determinado su posición con alta confianza.

## Estructura del Paquete

- **aidguide_04_slam/**: Contiene los nodos Python
  - `slam_node.py`: Nodo para interactuar con slam_toolbox
  - `localization_node.py`: Nodo para gestionar la localización
- **launch/**: Archivos de lanzamiento
  - `slam_launch.py`: Lanza el sistema SLAM
  - `localization_launch.py`: Lanza el sistema de localización
- **config/**: Archivos de configuración
  - `slam_params.yaml`: Parámetros para slam_toolbox
  - `amcl_params.yaml`: Parámetros para AMCL

## Personalización

Los parámetros de SLAM y localización pueden ajustarse modificando los archivos de configuración en la carpeta `config/`:

- `slam_params.yaml`: Ajusta parámetros como la resolución del mapa, distancia entre escaneos, etc.
- `amcl_params.yaml`: Ajusta parámetros de localización como el número de partículas, límites del láser, etc.

## Licencia

MIT

---

Para más información, contacte con el mantenedor del paquete. 