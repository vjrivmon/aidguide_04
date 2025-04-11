#!/bin/bash

# =====================================================
# Script para iniciar el sistema de monitoreo para AidGuide 04
# Este script inicia todos los componentes necesarios para
# monitorear el hardware, la batería, la temperatura y los logs del robot.
# =====================================================

# Colores para la salida
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Función para verificar si un comando existe
check_command_exists() {
    if ! command -v $1 &> /dev/null; then
        echo -e "${RED}Error: El comando $1 no está instalado.${NC}"
        exit 1
    fi
}

# Función para verificar si un nodo ROS está activo
check_node_active() {
    if ros2 node list | grep -q "$1"; then
        echo -e "- ${GREEN}Nodo $1 activo${NC}"
        return 0
    else
        echo -e "- ${RED}Nodo $1 no encontrado${NC}"
        return 1
    fi
}

# Función para verificar si un tema de ROS está activo
check_topic_active() {
    if ros2 topic list | grep -q "$1"; then
        echo -e "- ${GREEN}Tema $1 activo${NC}"
        return 0
    else
        echo -e "- ${RED}Tema $1 no encontrado${NC}"
        return 1
    fi
}

echo "============================================"
echo "    Iniciando Sistema de Monitoreo"
echo "============================================"

# Verificar que ROS2 esté instalado
check_command_exists ros2

# Verificar que el entorno de ROS2 esté configurado
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Advertencia: Variable ROS_DISTRO no definida. Intentando cargar el entorno...${NC}"
    if [ -f "/opt/ros/galactic/setup.bash" ]; then
        source /opt/ros/galactic/setup.bash
    else
        echo -e "${RED}Error: No se pudo encontrar el archivo setup.bash de ROS2.${NC}"
        exit 1
    fi
fi

# Ir al directorio del workspace
cd aidguide_04_ws

# Cargar el entorno del workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${RED}Error: No se pudo encontrar install/setup.bash en el workspace.${NC}"
    exit 1
fi

# Verificar que el paquete de monitoreo esté instalado
if ! ros2 pkg list | grep -q "aidguide_04_robot_monitoring"; then
    echo -e "${YELLOW}El paquete aidguide_04_robot_monitoring no está instalado. Intentando construirlo...${NC}"
    colcon build --packages-select aidguide_04_robot_monitoring
    if [ $? -ne 0 ]; then
        echo -e "${RED}Error al construir el paquete de monitoreo.${NC}"
        exit 1
    fi
    source install/setup.bash
fi

echo -e "${GREEN}Iniciando el sistema de monitoreo...${NC}"

# Lanzar el sistema de monitoreo
ros2 launch aidguide_04_robot_monitoring monitoring.launch.py &
MONITOR_PID=$!

# Esperar a que los nodos se inicialicen
echo "Esperando a que los nodos se inicialicen..."
sleep 5

# Verificar que todos los nodos estén en ejecución
echo -e "\n${YELLOW}Verificando nodos activos:${NC}"
NODE_STATUS=0
check_node_active "/battery_monitor" || NODE_STATUS=1
check_node_active "/hardware_monitor" || NODE_STATUS=1
check_node_active "/temperature_monitor" || NODE_STATUS=1
check_node_active "/log_monitor" || NODE_STATUS=1

# Verificar que todos los temas estén disponibles
echo -e "\n${YELLOW}Verificando temas activos:${NC}"
TOPIC_STATUS=0
check_topic_active "/battery_status" || TOPIC_STATUS=1
check_topic_active "/temperature_status" || TOPIC_STATUS=1
check_topic_active "/hardware_health" || TOPIC_STATUS=1
check_topic_active "/robot_logs" || TOPIC_STATUS=1

# Verificar el estado general
echo -e "\n============================================"
if [ $NODE_STATUS -eq 0 ] && [ $TOPIC_STATUS -eq 0 ]; then
    echo -e "${GREEN}Estado de la Conexión de Monitoreo: TODOS LOS SISTEMAS ACTIVOS${NC}"
    echo -e "El sistema está funcionando correctamente."
else
    echo -e "${RED}Estado de la Conexión de Monitoreo: ALGUNOS SISTEMAS INACTIVOS${NC}"
    echo -e "Hay problemas con la inicialización del sistema de monitoreo."
fi
echo -e "============================================"

echo -e "\nPara detener el monitoreo, presiona Ctrl+C o ejecuta: kill $MONITOR_PID"
echo -e "Para ver la interfaz web de monitoreo, abre http://localhost:3000/admin/dashboard"

# Esperar a que el usuario termine el proceso
wait $MONITOR_PID 