# AidGuide 04

Sistema de guiado asistido para robots móviles basado en ROS2 Galactic con interfaz web para monitorización y control.

## 🌟 Descripción General
El proyecto consta de dos componentes principales:
1. **Backend ROS2**: Sistema de control y guiado del robot
2. **Frontend Web**: Interfaz de usuario moderna para la monitorización y control del robot

## 🏗️ Estructura del Proyecto
El proyecto está organizado en los siguientes componentes:
- `src/custom_interface`: Interfaces personalizadas para la comunicación entre nodos
- `src/aidguide_04`: Nodo principal del sistema de guiado
- `src/aidguide_04_web`: Aplicación web frontend (Next.js)

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
rosdep install --from-paths src --ignore-src -r -y
```

3. Compilar el workspace:
```bash
colcon build
```

### Frontend Web
El frontend se puede iniciar de dos maneras:

#### Usando los scripts de automatización:

Para Windows:
```powershell
.\start-frontend.ps1
```

Para Linux/Unix:
```bash
chmod +x start-frontend.sh
./start-frontend.sh
```

#### Manualmente:
```bash
cd aidguide_04_ws/src/aidguide_04_web
npm install --legacy-peer-deps
npm run dev
```

## 💻 Uso del Sistema

### Iniciar el Sistema Completo

1. En una terminal, inicia el backend de ROS2:
```bash
ros2 launch aidguide_04 main.launch.py
```

2. En otra terminal, inicia el frontend web usando uno de los métodos descritos anteriormente.

3. Accede a la aplicación web:
   - URL: http://localhost:3000
   - La interfaz web te permitirá:
     - Visualizar el estado del robot en tiempo real
     - Controlar el movimiento del robot
     - Monitorizar sensores y sistemas
     - Configurar parámetros de navegación

## 👥 Equipo de Desarrollo
- Vicente Rivas Monferrer (Responsable) - vjrivmon@epsg.upv.es
- Irene Medina García
- Mimi Vladeva
- Hugo Belda Revert
- Marc Vilagrosa Caturla

## 🤝 Contribución
Por favor, lee [CONTRIBUTING.md](docs/CONTRIBUTING.md) para detalles sobre nuestro código de conducta y el proceso para enviar pull requests.

## 📄 Licencia
Este proyecto está bajo la Licencia MIT - ver el archivo [LICENSE](LICENSE) para más detalles.

## 🔧 Solución de Problemas Comunes

### Frontend
1. Si encuentras problemas con las dependencias:
   ```bash
   rm -rf node_modules
   npm cache clean --force
   ```
   Luego vuelve a ejecutar el script de inicio correspondiente.

2. Si el puerto 3000 está ocupado:
   - La aplicación intentará usar el siguiente puerto disponible automáticamente
   - Verifica la URL correcta en la consola al iniciar el frontend

### Backend
1. Problemas de conexión ROS2:
   ```bash
   source /opt/ros/galactic/setup.bash
   source install/setup.bash
   ```
   Luego intenta reiniciar el sistema. 