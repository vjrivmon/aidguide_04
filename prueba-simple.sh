#!/bin/bash

# Colores para mensajes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

# Función para imprimir mensajes con formato
print_message() {
    echo -e "${2}${1}${NC}"
}

# Cabecera
clear
print_message "╔════════════════════════════════════════════════════════════╗" "$CYAN"
print_message "║                                                            ║" "$CYAN"
print_message "║  🧪 AIDGUIDE 04 - PRUEBAS AUTOMATIZADAS                    ║" "$CYAN"
print_message "║                                                            ║" "$CYAN"
print_message "╚════════════════════════════════════════════════════════════╝" "$CYAN"
echo ""

# Función para verificar si un comando existe
check_command() {
    if ! command -v $1 &> /dev/null; then
        print_message "❌ El comando $1 no está instalado" "$RED"
        return 1
    fi
    return 0
}

# Determinar la ruta del workspace
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
if [[ $SCRIPT_DIR == *"aidguide_04"* ]]; then
    WORKSPACE_PATH="${SCRIPT_DIR}"
else
    WORKSPACE_PATH="${HOME}/aidguide_04"
fi

print_message "📂 Workspace detectado: ${WORKSPACE_PATH}" "$CYAN"

# Verificar que ROS2 está en el PATH
if [[ ! ":$PATH:" == *":opt/ros/galactic/bin:"* ]]; then
    if [ -f "/opt/ros/galactic/setup.bash" ]; then
        print_message "Configurando entorno ROS2..." "$YELLOW"
        source /opt/ros/galactic/setup.bash
    else
        print_message "❌ No se encontró ROS2 Galactic" "$RED"
        exit 1
    fi
fi

# Verificar que el script de pruebas existe
TEST_SCRIPT="${WORKSPACE_PATH}/aidguide_04_ws/src/aidguide_04/test/test_aidguide_system.py"
if [ ! -f "$TEST_SCRIPT" ]; then
    print_message "❌ No se encontró el script de pruebas en $TEST_SCRIPT" "$RED"
    exit 1
fi

# Verificar si ROS2 está en ejecución
if ! ros2 topic list &> /dev/null; then
    print_message "⚠️ ROS2 no parece estar en ejecución" "$YELLOW"
    print_message "   Se recomienda ejecutar primero start-ros2-gazebo.sh en otra terminal" "$YELLOW"
    read -p "¿Deseas continuar de todas formas? (s/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Ss]$ ]]; then
        print_message "Ejecución cancelada" "$YELLOW"
        exit 1
    fi
fi

# Verificar que Python está instalado
check_command python3 || { 
    print_message "❌ Python3 no está instalado" "$RED"
    exit 1
}

# Asegurar permisos de ejecución del script
chmod +x "$TEST_SCRIPT"

# Ejecutar las pruebas directamente con python
print_message "🚀 Iniciando pruebas automatizadas..." "$GREEN"
cd "${WORKSPACE_PATH}/aidguide_04_ws"
source install/setup.bash &> /dev/null || true

# Ejecutar el script de pruebas
python3 "${WORKSPACE_PATH}/aidguide_04_ws/src/aidguide_04/test/test_aidguide_system.py"

# Verificar el resultado
if [ $? -eq 0 ]; then
    print_message "✅ Todas las pruebas completadas con éxito" "$GREEN"
else
    print_message "❌ Algunas pruebas han fallado" "$RED"
    exit 1
fi

print_message "✅ Prueba finalizada" "$GREEN" 