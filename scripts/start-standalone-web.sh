#!/bin/bash

# Script para iniciar la aplicación web sin depender de ROS2
# Fecha: 2024

# Colores para los mensajes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${YELLOW}🚀 AIDGUIDE 04 - MODO STANDALONE WEB ${CYAN}                  ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Verificar si estamos en el directorio correcto
if [ ! -d "./aidguide_04_ws" ]; then
    echo -e "${RED}❌ No se encuentra el directorio del workspace${NC}"
    echo -e "${YELLOW}📝 Asegúrate de estar en el directorio raíz del proyecto${NC}"
    exit 1
fi

# Verificar si el mapa está en el lugar correcto
WORKSPACE_PATH="$(pwd)/aidguide_04_ws"
WEB_PUBLIC_PATH="$WORKSPACE_PATH/src/aidguide_04_web/public"
MAP_SOURCE_PATH="$WORKSPACE_PATH/src/aidguide_04_provide_map/map/aidguide_04_map.pgm"
MAP_TARGET_PATH="$WEB_PUBLIC_PATH/aidguide_04_map.png"

echo -e "${YELLOW}📂 Verificando mapas...${NC}"

# Verificar si el mapa PNG ya existe
if [ ! -f "$MAP_TARGET_PATH" ]; then
    echo -e "${YELLOW}🔍 El mapa PNG no existe, verificando imagen PGM original...${NC}"
    
    # Verificar si existe el mapa PGM original
    if [ -f "$MAP_SOURCE_PATH" ]; then
        echo -e "${YELLOW}📦 Copiando el mapa PGM al directorio público...${NC}"
        cp "$MAP_SOURCE_PATH" "$WEB_PUBLIC_PATH/"
        
        # Verificar si ImageMagick está instalado
        if command -v convert &> /dev/null; then
            echo -e "${YELLOW}🔄 Convirtiendo el mapa PGM a PNG...${NC}"
            cd "$WEB_PUBLIC_PATH" && convert aidguide_04_map.pgm aidguide_04_map.png
            echo -e "${GREEN}✅ Mapa convertido correctamente${NC}"
        else
            echo -e "${YELLOW}⚠️ ImageMagick no está instalado, intentando instalarlo...${NC}"
            sudo apt update && sudo apt install -y imagemagick-6.q16
            
            # Intentar convertir después de instalar
            cd "$WEB_PUBLIC_PATH" && convert aidguide_04_map.pgm aidguide_04_map.png
            if [ $? -eq 0 ]; then
                echo -e "${GREEN}✅ Mapa convertido correctamente${NC}"
            else
                echo -e "${RED}❌ No se pudo convertir el mapa. La aplicación usará el mapa de demostración.${NC}"
            fi
        fi
    else
        echo -e "${YELLOW}⚠️ No se encontró el mapa PGM original. La aplicación usará el mapa de demostración.${NC}"
    fi
else
    echo -e "${GREEN}✅ El mapa PNG ya existe${NC}"
fi

# Verificar si el archivo YAML está en el lugar correcto
YAML_SOURCE_PATH="$WORKSPACE_PATH/src/aidguide_04_provide_map/map/aidguide_04_map.yaml"
YAML_TARGET_PATH="$WEB_PUBLIC_PATH/aidguide_04_map.yaml"

if [ ! -f "$YAML_TARGET_PATH" ]; then
    echo -e "${YELLOW}🔍 El archivo YAML no existe en la carpeta pública, copiándolo...${NC}"
    
    if [ -f "$YAML_SOURCE_PATH" ]; then
        cp "$YAML_SOURCE_PATH" "$WEB_PUBLIC_PATH/"
        
        # Modificar la ruta de la imagen en el YAML
        sed -i 's|^image:.*$|image: aidguide_04_map.png|' "$YAML_TARGET_PATH"
        echo -e "${GREEN}✅ Archivo YAML copiado y modificado correctamente${NC}"
    else
        echo -e "${YELLOW}⚠️ No se encontró el archivo YAML original. Creando uno básico...${NC}"
        cat > "$YAML_TARGET_PATH" << EOF
image: aidguide_04_map.png
mode: trinary
resolution: 0.05
origin: [-8.16, -8.97, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
EOF
        echo -e "${GREEN}✅ Archivo YAML creado correctamente${NC}"
    fi
fi

# Iniciar la aplicación web
echo -e "${CYAN}🚀 Iniciando la aplicación web...${NC}"
cd "$WORKSPACE_PATH/src/aidguide_04_web" && npm run dev

# Si el script llega aquí, significa que la aplicación web se cerró
echo -e "${YELLOW}👋 La aplicación web se ha cerrado${NC}"
exit 0 