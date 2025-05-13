# Scripts para AidGuide 04

Este directorio contiene todos los scripts necesarios para ejecutar los diferentes componentes del proyecto AidGuide 04.

## Scripts Principales

- **start-ros2-gazebo.sh**: Script principal que lanza todo el entorno ROS2 y Gazebo. Configura e inicia todos los componentes necesarios para la simulación, navegación y visualización.

- **start-frontend.sh**: Inicia el servidor de desarrollo del frontend web. Configura automáticamente Node.js y npm si es necesario.

- **start-monitor.sh**: Inicia el sistema de monitoreo de hardware, batería, temperatura y logs del robot.

- **start-project.sh**: Script general para iniciar el proyecto completo, incluyendo todos los componentes.

- **start-web-with-chatbot.sh**: Inicia la aplicación web con soporte de chatbot basado en Ollama.

- **start-standalone-web.sh**: Inicia solo la aplicación web sin depender de ROS2, útil para desarrollo.

## Scripts de Utilidad

- **restart-ollama.sh**: Reinicia el servicio Ollama para el chatbot.

- **prueba-simple.sh**: Ejecuta pruebas automatizadas para verificar la funcionalidad básica del sistema.

- **start-services.sh**: Inicia los servicios de backend y frontend del proyecto.

## Scripts de SLAM y Navegación

La carpeta `slam/` contiene scripts específicos para operaciones de SLAM (Simultaneous Localization and Mapping):

- **slam/aidguide_slam_auto.sh**: Asistente automatizado para ejecutar tareas de SLAM y localización.

- **slam/instalar.sh**: Script de instalación para el asistente de SLAM.

## Uso

La mayoría de los scripts deben ejecutarse desde el directorio raíz del proyecto:

```bash
# Ejecutar el script principal
./scripts/start-ros2-gazebo.sh

# Iniciar solo el frontend
./scripts/start-frontend.sh

# Iniciar el monitor del sistema
./scripts/start-monitor.sh
```

Para los scripts de SLAM, primero debes hacer que sean ejecutables:

```bash
chmod +x scripts/slam/aidguide_slam_auto.sh
chmod +x scripts/slam/instalar.sh
```

Y luego ejecutarlos:

```bash
./scripts/slam/aidguide_slam_auto.sh
```

## Notas Importantes

- El script **start-ros2-gazebo.sh** es el más importante y lanza toda la aplicación con todos sus componentes.
- Algunos scripts pueden requerir privilegios de administrador para instalar dependencias.
- Asegúrate de tener ROS2 Galactic configurado correctamente antes de ejecutar los scripts. 