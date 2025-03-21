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

# Función para instalar curl si no está instalado
install_curl() {
    if ! command -v curl &> /dev/null; then
        echo -e "${YELLOW}📦 Instalando curl...${NC}"
        if command -v apt &> /dev/null; then
            sudo apt update && sudo apt install -y curl
        elif command -v dnf &> /dev/null; then
            sudo dnf install -y curl
        elif command -v yum &> /dev/null; then
            sudo yum install -y curl
        else
            echo -e "${RED}❌ No se pudo instalar curl. Por favor, instálalo manualmente.${NC}"
            exit 1
        fi
    fi
}

# Función para instalar nvm y Node.js
install_node() {
    # Instalar nvm si no está instalado
    if [ ! -d "$HOME/.nvm" ]; then
        echo -e "${YELLOW}📦 Instalando nvm...${NC}"
        install_curl
        curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash

        # Cargar nvm
        export NVM_DIR="$HOME/.nvm"
        [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
        [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"
    else
        # Cargar nvm si ya está instalado
        export NVM_DIR="$HOME/.nvm"
        [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
    fi

    # Verificar si nvm se instaló correctamente
    if ! command -v nvm &> /dev/null; then
        echo -e "${RED}❌ Error al instalar nvm. Intentando método alternativo...${NC}"
        if command -v apt &> /dev/null; then
            curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash -
            sudo apt install -y nodejs
        else
            echo -e "${RED}❌ No se pudo instalar Node.js. Por favor, instálalo manualmente.${NC}"
            exit 1
        fi
    else
        # Instalar la versión correcta de Node.js
        echo -e "${YELLOW}📦 Instalando Node.js v16...${NC}"
        nvm install 16
        nvm use 16
    fi
}

# Verificar si npm está instalado con la versión correcta
if ! command -v npm &> /dev/null; then
    echo -e "${YELLOW}📦 npm no está instalado. Instalando Node.js y npm...${NC}"
    install_node
else
    # Verificar la versión de Node.js
    NODE_VERSION=$(node -v | cut -d'v' -f2)
    REQUIRED_VERSION="16.0.0"

    if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$NODE_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
        echo -e "${YELLOW}📦 Actualizando Node.js a la versión requerida...${NC}"
        install_node
    else
        echo -e "${GREEN}✅ Versión de Node.js compatible: ${NODE_VERSION}${NC}"
    fi
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

# Limpiar caché de npm si existe node_modules con problemas
if [ -d "node_modules" ] && [ ! -f "node_modules/.package-lock.json" ]; then
    echo -e "${YELLOW}🧹 Limpiando instalación anterior...${NC}"
    rm -rf node_modules package-lock.json
    npm cache clean --force
fi

# Verificar si node_modules existe y está actualizado
if [ ! -d "node_modules" ] || [ ! -f "node_modules/.package-lock.json" ]; then
    echo -e "${YELLOW}📦 Instalando dependencias...${NC}"
    # Intentar diferentes estrategias de instalación
    npm install --legacy-peer-deps || {
        echo -e "${YELLOW}⚠️ Primer intento fallido, intentando con --force...${NC}"
        npm install --force || {
            echo -e "${YELLOW}⚠️ Segundo intento fallido, intentando limpiar caché...${NC}"
            npm cache clean --force
            npm install --legacy-peer-deps --no-package-lock || {
                echo -e "${RED}❌ Error al instalar las dependencias${NC}"
                exit 1
            }
        }
    }
else
    echo -e "${GREEN}✅ Las dependencias ya están instaladas${NC}"
fi

# Iniciar el servidor de desarrollo
echo -e "${CYAN}🌐 Iniciando el servidor de desarrollo...${NC}"
npm run dev || {
    echo -e "${RED}❌ Error al iniciar el servidor de desarrollo${NC}"
    echo -e "${YELLOW}📝 Intenta los siguientes pasos:${NC}"
    echo -e "${YELLOW}1. Eliminar node_modules y package-lock.json${NC}"
    echo -e "${YELLOW}2. Ejecutar: npm cache clean --force${NC}"
    echo -e "${YELLOW}3. Ejecutar el script nuevamente${NC}"
    exit 1
} 