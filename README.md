# 🤖 AidGuide 04

Sistema de guiado asistido para robots móviles basado en ROS2 Galactic con interfaz web para monitorización y control.

<div align="center">
  <img src="https://img.shields.io/badge/ROS2-Galactic-blue?style=for-the-badge" alt="ROS2 Galactic"/>
  <img src="https://img.shields.io/badge/Python-3.8+-green?style=for-the-badge" alt="Python 3.8+"/>
  <img src="https://img.shields.io/badge/Next.js-Frontend-black?style=for-the-badge" alt="Next.js"/>
</div>

## 📑 Contenidos

- [🌟 Descripción General](#-descripción-general)
- [🏗️ Estructura del Proyecto](#-estructura-del-proyecto)
- [📋 Requisitos](#-requisitos)
- [🚀 Instalación y Configuración](#-instalación-y-configuración)
- [💻 Uso del Sistema](#-uso-del-sistema)
- [⚙️ Scripts Automáticos](#-scripts-automáticos)
- [👥 Equipo de Desarrollo](#-equipo-de-desarrollo)
- [🔧 Solución de Problemas Comunes](#-solución-de-problemas-comunes)

## 🌟 Descripción General

El proyecto consta de dos componentes principales:
1. **Backend ROS2**: Sistema de control y guiado del robot
2. **Frontend Web**: Interfaz de usuario moderna para la monitorización y control del robot

## 🏗️ Estructura del Proyecto

```
aidguide_04/
├── aidguide_04_ws/
│   └── src/
│       ├── aidguide_04/                   # Nodo principal del sistema de guiado
│       ├── aidguide_04_web/               # Aplicación web frontend (Next.js)
│       ├── aidguide_04_world/             # Modelos y mundos para simulación
│       ├── aidguide_04_slam/              # Funcionalidades de SLAM
│       ├── aidguide_04_nav/               # Sistema de navegación base
│       ├── aidguide_04_nav_punto_a_punto/ # Navegación punto a punto
│       ├── aidguide_04_provide_map/       # Provisión de mapas
│       └── aidguide_04_my_nav2_system/    # Sistema personalizado Nav2
├── prueba-simple.sh                # Script para pruebas simples
├── start-frontend.ps1              # Script PowerShell para iniciar frontend
├── start-frontend.sh               # Script Bash para iniciar frontend
├── start-nav.ps1                   # Script PowerShell para iniciar navegación
└── start-ros2-gazebo.sh            # Script para iniciar ROS2 con Gazebo
```

## 📋 Requisitos

### Backend ROS2
- ROS2 Galactic
- Python 3.8+
- Dependencias específicas listadas en package.xml

### Frontend Web
- Node.js 16.x o superior
- npm 8.x o superior

## 🚀 Instalación y Configuración

### Backend ROS2
1. Clonar el repositorio:
```bash
git clone https://github.com/vjrivmon/aidguide_04.git
cd aidguide_04
```

2. Instalar dependencias de ROS2:
```bash
rosdep install --from-paths aidguide_04_ws/src --ignore-src -r -y
```

3. Compilar el workspace:
```bash
cd aidguide_04_ws
colcon build
```

## ⚙️ Scripts Automáticos

El proyecto incluye varios scripts para automatizar tareas comunes:

| Script | Plataforma | Descripción |
|--------|------------|-------------|
| `start-frontend.sh` | Linux/Unix | Inicia la aplicación web frontend |
| `start-frontend.ps1` | Windows | Inicia la aplicación web frontend |
| `start-nav.ps1` | Windows | Inicia el sistema de navegación |
| `start-ros2-gazebo.sh` | Linux/Unix | Inicia ROS2 con el simulador Gazebo |
| `prueba-simple.sh` | Linux/Unix | Ejecuta pruebas básicas del sistema |

### Ejecutar Scripts en Linux/Unix:
```bash
# Dar permisos de ejecución
chmod +x start-frontend.sh
chmod +x start-ros2-gazebo.sh
chmod +x prueba-simple.sh

# Ejecutar scripts
./start-frontend.sh
./start-ros2-gazebo.sh
./prueba-simple.sh
```

### Ejecutar Scripts en Windows:
```powershell
# Ejecutar scripts de PowerShell
.\start-frontend.ps1
.\start-nav.ps1
```

## 💻 Uso del Sistema

### Iniciar el Sistema Completo

1. **Iniciar la simulación**:
   ```bash
   # Linux/Unix
   ./start-ros2-gazebo.sh
   
   # Windows (necesitarás WSL o una solución similar para Gazebo)
   ```

2. **Iniciar el sistema de navegación**:
   ```bash
   # Linux/Unix (dentro del workspace)
   cd aidguide_04_ws
   source install/setup.bash
   ros2 launch aidguide_04_nav navigation.launch.py
   
   # Windows
   .\start-nav.ps1
   ```

3. **Iniciar el frontend web**:
   ```bash
   # Linux/Unix
   ./start-frontend.sh
   
   # Windows
   .\start-frontend.ps1
   ```

4. **Acceder a la aplicación web**:
   - URL: http://localhost:3000
   - Usar las siguientes credenciales según el rol:

   | Rol | Email | Contraseña |
   |-----|-------|------------|
   | Usuario ciego | user@aidguide.com | user123 |
   | Familiar | family@aidguide.com | family123 |
   | Administrador | admin@aidguide.com | admin123 |

## 👥 Equipo de Desarrollo
- Vicente Rivas Monferrer (Responsable) - vjrivmon@epsg.upv.es
- Irene Medina García
- Mimi Vladeva
- Hugo Belda Revert
- Marc Vilagrosa Caturla

## 🔧 Solución de Problemas Comunes

### Frontend
1. **Problemas con las dependencias**:
   ```bash
   cd aidguide_04_ws/src/aidguide_04_web
   rm -rf node_modules
   npm cache clean --force
   npm install --legacy-peer-deps
   npm run dev
   ```

2. **Puerto 3000 ocupado**:
   - La aplicación intentará usar el siguiente puerto disponible automáticamente
   - Verifica la URL correcta en la consola al iniciar el frontend

### Backend
1. **Problemas de conexión ROS2**:
   ```bash
   source /opt/ros/galactic/setup.bash
   cd aidguide_04_ws
   source install/setup.bash
   ```

2. **Errores en la simulación de Gazebo**:
   - Asegúrate de que todos los modelos están correctamente instalados
   - Reinicia el simulador con:
   ```bash
   killall -9 gzserver gzclient
   ./start-ros2-gazebo.sh
   ```
