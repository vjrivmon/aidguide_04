# Guía Rápida para AidGuide 04 SLAM

Esta guía proporciona instrucciones paso a paso para utilizar el paquete `aidguide_04_slam` para SLAM (Simultaneous Localization and Mapping) y localización.

## Requisitos Previos

1. Tener ROS2 Galactic instalado
2. Tener instalados los siguientes paquetes:
   ```bash
   sudo apt install ros-galactic-slam-toolbox ros-galactic-nav2-map-server ros-galactic-nav2-amcl ros-galactic-nav2-lifecycle-manager ros-galactic-turtlebot3-gazebo
   ```
3. Configurar la variable de entorno para el modelo del robot:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```

## Opciones Disponibles

El paquete ofrece varias funcionalidades:

1. **Ver un mapa existente**:
   ```bash
   ros2 launch aidguide_04_slam view_map_launch.py map_file:=<ruta_al_mapa>
   ```

2. **Localización en un mapa existente**:
   ```bash
   ros2 launch aidguide_04_slam localization_launch.py map_file:=<ruta_al_mapa>
   ```

3. **Prueba de localización con un robot simulado**:
   ```bash
   ros2 launch aidguide_04_slam test_map_with_robot.launch.py map_file:=<ruta_al_mapa>
   ```

4. **Crear un nuevo mapa con SLAM**:
   ```bash
   ros2 launch aidguide_04_slam test_slam.launch.py
   ```

## Mapas Disponibles

El paquete incluye los siguientes mapas:

1. **tb3_house_map**: Un mapa de una casa simulada para Turtlebot3
   ```bash
   ros2 launch aidguide_04_slam test_map_with_robot.launch.py map_file:=$(ros2 pkg prefix aidguide_04_slam)/share/aidguide_04_slam/maps/tb3_house_map.yaml
   ```

2. **turtlebot3_world**: Un mapa del mundo estándar de Turtlebot3
   ```bash
   ros2 launch aidguide_04_slam test_map_with_robot.launch.py map_file:=$(ros2 pkg prefix aidguide_04_slam)/share/aidguide_04_slam/maps/turtlebot3_world.yaml
   ```

## Paso a Paso: Prueba de Localización

1. Asegúrate de tener la variable de entorno correctamente configurada:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```

2. Inicia el entorno de simulación con localización:
   ```bash
   ros2 launch aidguide_04_slam test_map_with_robot.launch.py
   ```

3. Verás Gazebo y RViz abiertos. En RViz, puedes usar la herramienta "2D Pose Estimate" para establecer la posición inicial del robot.

4. Para mover el robot, puedes usar:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

5. Para comprobar el estado de la localización:
   ```bash
   ros2 topic echo /localization/status
   ```

## Paso a Paso: Crear un Nuevo Mapa

1. Inicia el entorno de SLAM:
   ```bash
   ros2 launch aidguide_04_slam test_slam.launch.py
   ```

2. Mueve el robot por el entorno para crear el mapa:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. Una vez completado el mapa, guárdalo:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/mi_nuevo_mapa
   ```

4. Para usar este mapa en el futuro, cópialo a la carpeta de mapas del paquete:
   ```bash
   cp ~/mi_nuevo_mapa.* ~/aidguide_04/aidguide_04_ws/src/aidguide_04_slam/maps/
   ```

5. Recompila el paquete:
   ```bash
   cd ~/aidguide_04 && colcon build --packages-select aidguide_04_slam
   ```

## Solución de Problemas

- **No se ve el robot en RViz**: Asegúrate de que la transformación TF está publicada correctamente. Verifica que los frames configurados en los archivos YAML coinciden con los del robot.

- **El robot no se localiza**: Intenta establecer manualmente la posición inicial usando "2D Pose Estimate" en RViz.

- **Errores de tiempo de simulación**: Asegúrate de que `use_sim_time` está configurado correctamente en todos los nodos.

- **Mapas no encontrados**: Verifica la ruta a los archivos de mapas y asegúrate de que el paquete se ha compilado después de agregar nuevos mapas. 