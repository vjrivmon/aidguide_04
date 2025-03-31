#!/bin/bash
#
# Script de instalación para el asistente de SLAM
# Crea un enlace simbólico en /usr/local/bin para fácil acceso
#

ROJO='\033[0;31m'
VERDE='\033[0;32m'
AMARILLO='\033[0;33m'
AZUL='\033[0;34m'
NC='\033[0m' # Sin Color

# Detectar el directorio actual
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
SCRIPT_PATH="$SCRIPT_DIR/aidguide_slam_auto.sh"

# Verificar que el script existe
if [ ! -f "$SCRIPT_PATH" ]; then
    echo -e "${ROJO}Error: No se encuentra el script aidguide_slam_auto.sh${NC}"
    exit 1
fi

# Solicitar privilegios para crear el enlace en /usr/local/bin
echo -e "${AMARILLO}Se requiere privilegios sudo para crear un enlace en /usr/local/bin${NC}"
sudo ln -sf "$SCRIPT_PATH" /usr/local/bin/aidguide-slam

# Verificar que se creó el enlace
if [ -L "/usr/local/bin/aidguide-slam" ]; then
    echo -e "${VERDE}¡Instalación exitosa!${NC}"
    echo -e "${AZUL}Ahora puede ejecutar el asistente de SLAM desde cualquier ubicación con el comando:${NC}"
    echo -e "${VERDE}aidguide-slam${NC}"
else
    echo -e "${ROJO}Error: No se pudo crear el enlace simbólico.${NC}"
    echo -e "${AMARILLO}Puede usar el script directamente en:${NC}"
    echo -e "${VERDE}$SCRIPT_PATH${NC}"
fi

# Información adicional
echo -e "\n${AZUL}Para obtener ayuda, ejecute:${NC}"
echo -e "${VERDE}aidguide-slam help${NC}" 