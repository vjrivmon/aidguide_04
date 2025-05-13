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

        # Recargar el shell para asegurar que nvm está disponible
        echo -e "${YELLOW}📝 Recargando el shell para aplicar los cambios...${NC}"
        exec $SHELL
    else
        # Cargar nvm si ya está instalado
        export NVM_DIR="$HOME/.nvm"
        [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
    fi

    # Verificar si nvm se instaló correctamente
    if ! command -v nvm &> /dev/null; then
        echo -e "${RED}❌ Error al instalar nvm. Intentando método alternativo...${NC}"
        if command -v apt &> /dev/null; then
            curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
            sudo apt install -y nodejs
        else
            echo -e "${RED}❌ No se pudo instalar Node.js. Por favor, instálalo manualmente.${NC}"
            exit 1
        fi
    else
        # Instalar la versión LTS de Node.js 18
        echo -e "${YELLOW}📦 Instalando Node.js v18 LTS...${NC}"
        nvm install 18
        nvm use 18
        nvm alias default 18
    fi
}

# Verificar si npm está instalado con la versión correcta
if ! command -v npm &> /dev/null; then
    echo -e "${YELLOW}📦 npm no está instalado. Instalando Node.js y npm...${NC}"
    install_node
else
    # Verificar la versión de Node.js
    NODE_VERSION=$(node -v | cut -d'v' -f2)
    REQUIRED_VERSION="18.0.0"

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

# Verificar si las dependencias ya están instaladas
if [ -d "node_modules" ] && [ -f "package-lock.json" ]; then
    echo -e "${GREEN}✅ Dependencias ya instaladas. Verificando integridad...${NC}"
    
    # Verificar si hay algún problema con las dependencias instaladas
    if npm ls --depth=0 2>/dev/null | grep -q "ERR!"; then
        echo -e "${YELLOW}⚠️ Se detectaron problemas en las dependencias instaladas${NC}"
        echo -e "${YELLOW}📝 ¿Deseas reinstalar las dependencias? (s/n)${NC}"
        read -r respuesta
        if [[ "$respuesta" =~ ^[Ss]$ ]]; then
            echo -e "${YELLOW}🧹 Reinstalando dependencias...${NC}"
            rm -rf node_modules package-lock.json
            npm cache clean --force
            npm install --legacy-peer-deps
        fi
    else
        echo -e "${GREEN}✅ Las dependencias están correctamente instaladas${NC}"
    fi
else
    echo -e "${YELLOW}📦 Instalando dependencias por primera vez...${NC}"
    # Intentar diferentes estrategias de instalación
    npm install --legacy-peer-deps || {
        echo -e "${YELLOW}⚠️ Primer intento fallido, intentando con --force...${NC}"
        npm install --force || {
            echo -e "${YELLOW}⚠️ Segundo intento fallido, intentando limpiar caché...${NC}"
            npm cache clean --force
            npm install --legacy-peer-deps --no-package-lock || {
                echo -e "${RED}❌ Error al instalar las dependencias${NC}"
                echo -e "${YELLOW}📝 Por favor, intenta los siguientes pasos manualmente:${NC}"
                echo -e "${YELLOW}1. Ejecuta: nvm use 18${NC}"
                echo -e "${YELLOW}2. Ejecuta: npm cache clean --force${NC}"
                echo -e "${YELLOW}3. Ejecuta: npm install --legacy-peer-deps${NC}"
                exit 1
            }
        }
    }
fi

# Verificar que next está instalado correctamente
if ! npm list next | grep -q "next@"; then
    echo -e "${RED}❌ Next.js no se instaló correctamente${NC}"
    echo -e "${YELLOW}📝 Intentando instalar Next.js específicamente...${NC}"
    npm install next@latest react@latest react-dom@latest
fi

# Instalar tipos de d3 y resolver conflictos de dependencias
if ! npm list @types/d3 &> /dev/null; then
    echo -e "${YELLOW}📦 Instalando tipos de D3...${NC}"
    npm install --save-dev @types/d3 --legacy-peer-deps
    # Corregir versión de date-fns para compatibilidad
    npm install date-fns@^3.0.0 --legacy-peer-deps
fi

# Iniciar el servidor de desarrollo
echo -e "${CYAN}🌐 Iniciando el servidor de desarrollo...${NC}"
npm run dev || {
    echo -e "${RED}❌ Error al iniciar el servidor de desarrollo${NC}"
    echo -e "${YELLOW}📝 Verifica lo siguiente:${NC}"
    echo -e "${YELLOW}1. Versión de Node.js (debe ser 18 o superior):${NC}"
    node -v
    echo -e "${YELLOW}2. Versión de npm:${NC}"
    npm -v
    echo -e "${YELLOW}3. Contenido de package.json:${NC}"
    cat package.json
    echo -e "${YELLOW}4. Para reintentar:${NC}"
    echo -e "${YELLOW}   - Elimina node_modules, package-lock.json y .next${NC}"
    echo -e "${YELLOW}   - Ejecuta: nvm use 18${NC}"
    echo -e "${YELLOW}   - Ejecuta el script nuevamente${NC}"
    exit 1
} 