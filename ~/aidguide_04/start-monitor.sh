#!/bin/bash

# Script para iniciar el sistema completo de monitoreo del robot AidGuide 04
# Este script inicia todos los componentes necesarios para el monitoreo
# y verifica que estén funcionando correctamente.

echo "========================================================================"
echo "🔍 SISTEMA DE MONITOREO AIDGUIDE 04 - INICIANDO"
echo "========================================================================"

# Función para verificar si un comando existe
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Función para verificar si un nodo ROS está activo
check_ros_node() {
    ros2 node list | grep -q "$1"
    return $?
}

# Función para verificar si un tema ROS está activo
check_ros_topic() {
    ros2 topic list | grep -q "$1"
    return $?
}

# Verificar que ROS2 esté instalado
if ! command_exists ros2; then
    echo "❌ ERROR: ROS2 no está instalado o no está en el PATH."
    echo "   Por favor, asegúrate de tener ROS2 Galactic instalado y configurado."
    exit 1
fi

# Ir al directorio del workspace
cd ~/aidguide_04/aidguide_04_ws || {
    echo "❌ ERROR: No se puede acceder al directorio del workspace."
    exit 1
}

# Inicializar ROS2
echo "🔄 Inicializando entorno ROS2..."
source /opt/ros/galactic/setup.bash
source install/setup.bash

# Verificar que el paquete de monitoreo esté instalado
if ! ros2 pkg list | grep -q "aidguide_04_robot_monitoring"; then
    echo "⚠️ El paquete aidguide_04_robot_monitoring no está instalado o no se detecta."
    echo "🔄 Intentando reconstruir el paquete..."
    colcon build --packages-select aidguide_04_robot_monitoring
    source install/setup.bash
fi

# Iniciar el sistema de monitoreo
echo "🚀 Iniciando sistema de monitoreo del robot..."
ros2 launch aidguide_04_robot_monitoring monitoring.launch.py &
LAUNCH_PID=$!

# Esperar un momento para que los nodos se inicien
echo "⏳ Esperando a que los nodos se inicialicen..."
sleep 5

# Verificar que los nodos estén funcionando
echo "🔍 Verificando nodos activos..."
ros2 node list

# Verificar temas de monitoreo
echo "🔍 Verificando temas de monitoreo..."
ros2 topic list | grep -E 'battery|temperature|hardware|log'

echo "📊 Estado de la conexión de monitoreo:"
if check_ros_topic "/battery_status" && check_ros_topic "/temperature_sensor" && check_ros_topic "/hardware_health" && check_ros_topic "/log_messages"; then
    echo "✅ Todos los temas de monitoreo están activos y funcionando."
else
    echo "⚠️ Algunos temas de monitoreo no están activos."
fi

echo "========================================================================"
echo "📋 INSTRUCCIONES:"
echo "   - Los datos de monitoreo se mostrarán automáticamente en la consola."
echo "   - Presiona Ctrl+C una vez para detener el monitoreo."
echo "========================================================================"

# Mantener el script en ejecución para que los logs sean visibles
wait $LAUNCH_PID

echo "🛑 Sistema de monitoreo detenido." 