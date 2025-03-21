#!/bin/bash

# Script de automatización para iniciar el paquete de navegación aidguide_04_nav
# Autor: DevOps Team
# Fecha: 2024
# Descripción: Este script automatiza la compilación y ejecución del paquete de navegación

# Colores para los mensajes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${CYAN}🚀 Iniciando el proceso de configuración de navegación...${NC}"

# 1. Nos situamos en la carpeta del workspace
echo -e "${YELLOW}📂 Navegando al workspace...${NC}"
cd ~/aidguide_04/aidguide_04_ws

# 2. Compilamos el paquete
echo -e "${YELLOW}🔨 Compilando el paquete...${NC}"
colcon build --packages-select aidguide_04_nav

# 3. Verificamos y establecemos permisos de los archivos Python
echo -e "${YELLOW}🔑 Verificando permisos de archivos...${NC}"
NAV_PATH="src/aidguide_04_nav/aidguide_04_nav"
LAUNCH_PATH="src/aidguide_04_nav/launch"

# Verificar y establecer permisos para los archivos Python
if [ -f "$NAV_PATH/aidguide_04_nav.py" ]; then
    echo -e "${GREEN}✅ aidguide_04_nav.py encontrado${NC}"
    chmod +x "$NAV_PATH/aidguide_04_nav.py"
else
    echo -e "${RED}❌ No se encuentra aidguide_04_nav.py${NC}"
    exit 1
fi

if [ -f "$LAUNCH_PATH/aidguide_04_nav_launch.launch.py" ]; then
    echo -e "${GREEN}✅ Launch file encontrado${NC}"
    chmod +x "$LAUNCH_PATH/aidguide_04_nav_launch.launch.py"
else
    echo -e "${RED}❌ No se encuentra el archivo launch${NC}"
    exit 1
fi

# 4. Iniciamos la simulación en una nueva terminal
echo -e "${CYAN}🌐 Iniciando simulación Gazebo...${NC}"
gnome-terminal -- bash -c "ros2 launch turtlebot3_gazebo empty_world.launch.py" || \
xterm -e "ros2 launch turtlebot3_gazebo empty_world.launch.py" || \
konsole -e "ros2 launch turtlebot3_gazebo empty_world.launch.py"

# 5. Esperamos unos segundos para que Gazebo se inicie
echo -e "${YELLOW}⏳ Esperando a que Gazebo se inicie...${NC}"
sleep 10

# 6. Iniciamos la navegación
echo -e "${CYAN}🗺️ Iniciando sistema de navegación...${NC}"
source install/setup.bash
ros2 launch aidguide_04_nav aidguide_04_nav_launch.launch.py 