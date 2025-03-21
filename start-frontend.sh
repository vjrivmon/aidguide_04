#!/bin/bash

# Script de automatización para iniciar el frontend en Linux/Unix
# Autor: DevOps Team
# Fecha: 2024
# Descripción: Este script automatiza la instalación de dependencias y el inicio del servidor de desarrollo

# Colores para los mensajes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${CYAN}🚀 Iniciando el proceso de configuración del frontend...${NC}"

# Verificar si npm está instalado
if ! command -v npm &> /dev/null; then
    echo -e "${RED}❌ npm no está instalado. Intentando instalar...${NC}"
    if command -v apt &> /dev/null; then
        sudo apt update
        sudo apt install -y nodejs npm
    elif command -v dnf &> /dev/null; then
        sudo dnf install -y nodejs npm
    elif command -v yum &> /dev/null; then
        sudo yum install -y nodejs npm
    else
        echo -e "${RED}❌ No se pudo instalar npm automáticamente. Por favor, instálalo manualmente.${NC}"
        exit 1
    fi
fi

# Verificar la versión de Node.js
NODE_VERSION=$(node -v | cut -d'v' -f2)
REQUIRED_VERSION="16.0.0"

if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$NODE_VERSION" | sort -V | head -n1)" = "$REQUIRED_VERSION" ]; then
    echo -e "${GREEN}✅ Versión de Node.js compatible: ${NODE_VERSION}${NC}"
else
    echo -e "${RED}❌ Se requiere Node.js versión ${REQUIRED_VERSION} o superior. Versión actual: ${NODE_VERSION}${NC}"
    echo -e "${YELLOW}📝 Por favor, actualiza Node.js usando nvm o el gestor de paquetes de tu sistema${NC}"
    exit 1
fi

# Verificar si estamos en el directorio correcto
if [ ! -d "./aidguide_04_ws" ]; then
    echo -e "${RED}❌ No se encuentra el directorio del workspace${NC}"
    echo -e "${YELLOW}📝 Asegúrate de estar en el directorio raíz del proyecto${NC}"
    exit 1
fi

# Navegar al directorio del frontend
echo -e "${YELLOW}📂 Navegando al directorio del frontend...${NC}"
cd ./aidguide_04_ws/src/aidguide_04_web || {
    echo -e "${RED}❌ No se pudo acceder al directorio del frontend${NC}"
    exit 1
}

# Verificar si node_modules existe y está actualizado
if [ ! -d "node_modules" ] || [ ! -f "node_modules/.package-lock.json" ]; then
    echo -e "${YELLOW}📦 Instalando dependencias...${NC}"
    npm install --legacy-peer-deps || {
        echo -e "${RED}❌ Error al instalar las dependencias${NC}"
        exit 1
    }
else
    echo -e "${GREEN}✅ Las dependencias ya están instaladas${NC}"
fi

# Iniciar el servidor de desarrollo
echo -e "${CYAN}🌐 Iniciando el servidor de desarrollo...${NC}"
npm run dev || {
    echo -e "${RED}❌ Error al iniciar el servidor de desarrollo${NC}"
    echo -e "${YELLOW}📝 Intenta eliminar node_modules y package-lock.json, luego ejecuta el script nuevamente${NC}"
    exit 1
} 