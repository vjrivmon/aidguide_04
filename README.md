# 🤖 AidGuide 04

Sistema de guiado asistido para robots móviles basado en ROS2 Galactic con interfaz web para monitorización y control.

<div align="center">
  <img src="https://img.shields.io/badge/ROS2-Galactic-blue?style=for-the-badge" alt="ROS2 Galactic"/>
  <img src="https://img.shields.io/badge/Python-3.8+-green?style=for-the-badge" alt="Python 3.8+"/>
  <img src="https://img.shields.io/badge/Next.js-Frontend-black?style=for-the-badge" alt="Next.js"/>
</div>

## 🚀 Descripción del Proyecto
AidGuide 04 es un sistema robótico basado en ROS2 Galactic que integra simulación en Gazebo, backend con API REST y frontend web para control y visualización.

## 📋 Requisitos Previos

### Software Requerido
- **ROS2 Galactic**: Entorno de desarrollo para robótica
- **Gazebo**: Simulador 3D para robots
- **Docker & Docker Compose**: Para servicios de backend
- **Node.js** (v14 o superior): Para desarrollo frontend

### Paquetes ROS2 Adicionales
```bash
sudo apt install ros-galactic-rosbridge-server ros-galactic-web-video-server
```

## 🛠️ Estructura del Proyecto
```
aidguide_04/
├── aidguide_04_ws/            # Espacio de trabajo ROS2
│   ├── src/
│   │   ├── aidguide_04/               # Paquete principal
│   │   ├── aidguide_04_nav/           # Navegación
│   │   ├── aidguide_04_slam/          # SLAM
│   │   ├── aidguide_04_backend/       # Backend API REST
│   │   └── aidguide_04_web/           # Frontend
│   └── start-services.sh       # Script para iniciar servicios backend
├── start-project.sh           # Script principal (inicia todo)
├── start-ros2-gazebo.sh       # Inicia ROS2 y Gazebo
└── start-frontend.sh          # Inicia la aplicación frontend
```

## 🚀 Ejecución del Proyecto

### Método 1: Ejecución Completa Automatizada
El script principal inicia todos los componentes del proyecto en un solo paso:

```bash
# Dar permisos de ejecución si es necesario
chmod +x start-project.sh

# Ejecutar el script principal
./start-project.sh
```

Este script:
1. Verifica e inicia Docker si es necesario
2. Inicia el backend (MySQL, API) con Docker
3. Inicia ROS2 y Gazebo
4. Inicia la aplicación frontend

### Método 2: Ejecución por Componentes

Si prefieres iniciar componentes individualmente:

1. **Backend (API + Base de datos)**:
```bash
cd aidguide_04_ws
chmod +x start-services.sh
./start-services.sh
```

2. **ROS2 y Gazebo**:
```bash
chmod +x start-ros2-gazebo.sh
./start-ros2-gazebo.sh
```

3. **Frontend**:
```bash
chmod +x start-frontend.sh
./start-frontend.sh
```

## 🔗 Acceso a los Servicios

Una vez iniciado el proyecto, puedes acceder a los siguientes servicios:

- **Frontend**: http://localhost:3334
- **Backend API**: http://localhost:3333/api
- **Documentación API**: http://localhost:3333/api-docs
- **MySQL**: localhost:3306 (usuario: aiduser, contraseña: password123)

## 🔧 Solución de Problemas Comunes

### Problemas con Docker
- **Error "Docker daemon no está en ejecución"**: Asegúrate de iniciar Docker con `sudo systemctl start docker`
- **Error de permisos**: Añade tu usuario al grupo docker con `sudo usermod -aG docker $USER` y reinicia la sesión
- **Conflicto de puertos**: Si recibes errores sobre puertos ya en uso, verifica qué procesos están utilizando los puertos 3333 y 3334 con `sudo lsof -i :<puerto>`

### Problemas con ROS2
- **"Command not found: ros2"**: Asegúrate de tener ROS2 Galactic instalado y haber ejecutado `source /opt/ros/galactic/setup.bash`
- **Paquetes no encontrados**: Ejecuta `rosdep install --from-paths src --ignore-src -r -y` en el workspace

## 🧪 Pruebas y CI/CD

El proyecto utiliza GitHub Actions para realizar pruebas automáticas:

### Validación de Base de Datos
Este workflow verifica que la conexión a la base de datos funcione correctamente:

```yaml
# .github/workflows/db-validation.yml
name: Validación de Conexión a Base de Datos

on:
  push:
    branches: [ develop ]
  pull_request:
    branches: [ develop ]
```

Para que los tests funcionen correctamente, asegúrate de:

1. Configurar los siguientes secretos en GitHub:
   - `DB_USER`: Usuario de la base de datos
   - `DB_PASSWORD`: Contraseña de la base de datos 
   - `DB_NAME`: Nombre de la base de datos

2. Verificar que el archivo `verify-db-connection.js` esté actualizado en el repositorio.

Si encuentras errores en los tests de GitHub Actions:
- Verifica que todas las dependencias estén correctamente especificadas en `package.json`
- Asegúrate de que los scripts de prueba funcionan localmente antes de hacer push

## 🤝 Contribución

1. Asegúrate de tener la última versión del repositorio
2. Crea una rama para tu característica (`git checkout -b feature/nueva-caracteristica`)
3. Haz tus cambios y pruébalos localmente
4. Envía un pull request a la rama `develop`

## 📄 Licencia
[MIT License](LICENSE)

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
