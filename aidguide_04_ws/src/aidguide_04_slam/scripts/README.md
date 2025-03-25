# Asistente Automático para SLAM en AidGuide 04

Este directorio contiene scripts que facilitan el uso del sistema SLAM y localización en AidGuide 04.

## Descripción de los Scripts

- **aidguide_slam_auto.sh**: Script principal que proporciona un menú interactivo para ejecutar tareas de SLAM y localización.
- **instalar.sh**: Script de instalación que configura un enlace simbólico para acceder fácilmente al asistente desde cualquier ubicación.

## Requisitos

- ROS2 Galactic instalado
- Los paquetes de SLAM y navegación instalados (`slam_toolbox`, `nav2_map_server`, `nav2_amcl`, `nav2_lifecycle_manager`)
- Permisos de administrador para crear el enlace simbólico (solo si ejecuta `instalar.sh`)

## Instalación

1. Compile el paquete aidguide_04_slam:
   ```bash
   cd ~/aidguide_04
   colcon build --packages-select aidguide_04_slam
   source install/setup.bash
   ```

2. Ejecute el script de instalación (opcional, para crear un enlace global):
   ```bash
   ~/aidguide_04/install/aidguide_04_slam/share/aidguide_04_slam/scripts/instalar.sh
   ```

3. Una vez instalado, puede ejecutar el asistente desde cualquier ubicación con:
   ```bash
   aidguide-slam
   ```

## Uso

### Como Menú Interactivo

Ejecute el script sin argumentos para acceder al menú interactivo:
```bash
aidguide-slam
```

El menú ofrece las siguientes opciones:
1. Iniciar SLAM (crear nuevo mapa)
2. Guardar mapa actual
3. Ejecutar localización en mapa existente
4. Visualizar mapa existente
0. Salir

### Con Argumentos en Línea de Comandos

También puede ejecutar funciones específicas mediante argumentos:

- **Ejecutar SLAM**:
  ```bash
  aidguide-slam slam
  ```

- **Guardar un mapa**:
  ```bash
  aidguide-slam save
  ```

- **Localización en un mapa existente**:
  ```bash
  aidguide-slam localize
  ```

- **Visualizar un mapa existente**:
  ```bash
  aidguide-slam view
  ```

- **Mostrar ayuda**:
  ```bash
  aidguide-slam help
  ```

## Características

- ✓ Menú interactivo con interfaz amigable
- ✓ Verificación automática de dependencias
- ✓ Selección de mapas disponibles
- ✓ Guardado automático de mapas con copia al paquete
- ✓ Recompilación automática después de agregar mapas nuevos
- ✓ Configuración automática de TURTLEBOT3_MODEL si no está definido
- ✓ Mensajes con códigos de colores para mejorar la legibilidad

## Solución de Problemas

- **El script no encuentra los mapas**: Asegúrese de que los mapas estén en el directorio `maps/` y que el paquete esté compilado e instalado correctamente.

- **Error al ejecutar los comandos de ROS2**: Verifique que ROS2 esté correctamente instalado y que el entorno esté configurado.

- **El enlace simbólico no funciona**: Puede ejecutar el script directamente desde su ubicación en el paquete instalado.

## Desarrollo

Si desea modificar o mejorar estos scripts:

1. Edite los archivos en `~/aidguide_04/aidguide_04_ws/src/aidguide_04_slam/scripts/`
2. Recompile el paquete con `colcon build --packages-select aidguide_04_slam`
3. Vuelva a ejecutar el script de instalación si es necesario 