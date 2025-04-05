#!/bin/bash

# Script para iniciar todos los servicios de AidGuide (Frontend y Backend)
# Autor: Vicente
# Fecha: 2024

# Variables de configuración
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BACKEND_PATH="$SCRIPT_DIR/src/aidguide_04_backend"
FRONTEND_PATH="$SCRIPT_DIR/src/aidguide_04_web"
PROJECT_NAME="aidguide_04"
MAX_RETRIES=2

# Colores para la terminal
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Iconos para mejorar la visualización
ICON_CHECK="✅"
ICON_ERROR="❌"
ICON_WARNING="⚠️"
ICON_INFO="ℹ️"
ICON_FOLDER="📂"
ICON_ROCKET="🚀"
ICON_CLEANING="🧹"
ICON_DOCKER="🐳"
ICON_LOADING="⏳"
ICON_RETRY="🔄"
ICON_WEB="🌐"
ICON_DATABASE="🗄️"
ICON_DOCS="📚"

# Función para mostrar una barra de progreso
show_progress() {
    local duration=$1
    local message=$2
    local width=50
    local bar_char="▓"
    local empty_char="░"
    
    echo -ne "${YELLOW}${message}${NC}\n"
    for i in $(seq 1 $width); do
        echo -ne "${CYAN}${bar_char}${NC}"
        sleep $(echo "scale=3; $duration/$width" | bc)
    done
    echo -e " ${GREEN}${BOLD}¡Completado!${NC}"
}

# Función para mostrar el encabezado
show_header() {
    clear
    echo -e "\n${CYAN}${BOLD}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}${BOLD}║                 ${ICON_ROCKET} AIDGUIDE LAUNCHER ${ICON_ROCKET}                 ║${NC}"
    echo -e "${CYAN}${BOLD}╚════════════════════════════════════════════════════════════╝${NC}\n"
    echo -e "${BLUE}${BOLD}Iniciando servicios para el proyecto AidGuide...${NC}\n"
}

# Función para verificar si un directorio existe
verify_directory() {
    local path=$1
    local name=$2
    echo -ne "${YELLOW}${ICON_INFO} Verificando directorio ${BOLD}$name${NC}... "
    if [ ! -d "$path" ]; then
        echo -e "${RED}${ICON_ERROR} No encontrado${NC}"
        echo -e "${YELLOW}Directorio actual: $(pwd)${NC}"
        echo -e "${YELLOW}Directorios disponibles:${NC}"
        ls -la "$SCRIPT_DIR"
        return 1
    fi
    echo -e "${GREEN}${ICON_CHECK} Encontrado${NC}"
    return 0
}

# Mostrar el encabezado
show_header

# Verificar si los directorios existen
echo -e "${CYAN}${BOLD}▶ VERIFICANDO DIRECTORIOS${NC}"
if ! verify_directory "$BACKEND_PATH" "Backend"; then
    exit 1
fi

# Verificar el directorio del frontend
verify_directory "$FRONTEND_PATH" "Frontend" || true
echo

# Navegamos al directorio del backend donde está el docker-compose.yml
echo -e "${CYAN}${BOLD}▶ PREPARANDO ENTORNO${NC}"
echo -e "${YELLOW}${ICON_FOLDER} Navegando a ${BOLD}$BACKEND_PATH${NC}"
cd "$BACKEND_PATH" || { echo -e "${RED}${ICON_ERROR} Error: No se pudo acceder al directorio $BACKEND_PATH${NC}"; exit 1; }

# Comprobamos si Docker está en ejecución
echo -ne "${YELLOW}${ICON_INFO} Comprobando estado de Docker... ${NC}"
if ! docker info > /dev/null 2>&1; then
    echo -e "${RED}${ICON_ERROR} No en ejecución${NC}"
    echo -e "${RED}${BOLD}Docker no está en ejecución. Por favor, inicie Docker e intente nuevamente.${NC}"
    exit 1
else
    echo -e "${GREEN}${ICON_CHECK} En ejecución${NC}"
fi

# Verificar si existe el archivo docker-compose.yml
echo -ne "${YELLOW}${ICON_INFO} Verificando archivo docker-compose.yml... ${NC}"
if [ ! -f "docker-compose.yml" ]; then
    echo -e "${RED}${ICON_ERROR} No encontrado${NC}"
    echo -e "${RED}${BOLD}Error: No se encontró el archivo docker-compose.yml en $(pwd)${NC}"
    exit 1
else
    echo -e "${GREEN}${ICON_CHECK} Encontrado${NC}"
fi
echo

# Limpiar todos los contenedores relacionados con el proyecto
echo -e "${CYAN}${BOLD}▶ PREPARANDO CONTENEDORES${NC}"
echo -e "${YELLOW}${ICON_CLEANING} Limpiando contenedores existentes...${NC}"
docker-compose down > /dev/null 2>&1
containers=$(docker ps -a --filter "name=aidguide" --format "{{.Names}}")
if [ ! -z "$containers" ]; then
    echo -e "  ${YELLOW}${ICON_INFO} Eliminando contenedores adicionales...${NC}"
    docker rm -f $containers > /dev/null 2>&1
fi
show_progress 1 "${ICON_LOADING} Limpieza en progreso..."
echo

# Función para iniciar los contenedores y manejar reintentos
start_containers() {
    local retry=${1:-0}
    
    # Iniciamos los contenedores con docker-compose
    echo -e "${CYAN}${BOLD}▶ INICIANDO SERVICIOS (Intento ${retry+1}/${MAX_RETRIES+1})${NC}"
    echo -e "${YELLOW}${ICON_DOCKER} Creando y arrancando contenedores...${NC}"
    docker-compose up -d --build
    
    # Mostramos una barra de progreso mientras los contenedores se inician
    show_progress 3 "${ICON_LOADING} Inicializando servicios..."
    
    # Verificamos el estado de los contenedores
    echo -e "\n${MAGENTA}${BOLD}${ICON_INFO} ESTADO DE LOS SERVICIOS:${NC}"
    echo -e "${CYAN}╭───────────────────────────────────────────────────────────╮${NC}"
    docker-compose ps
    echo -e "${CYAN}╰───────────────────────────────────────────────────────────╯${NC}"
    
    # Verificar si el frontend está en ejecución
    if ! docker ps --filter "name=aidguide_frontend" --format "{{.Names}}" | grep -q "aidguide_frontend"; then
        echo -e "\n${YELLOW}${ICON_WARNING} El contenedor del frontend no se inició correctamente.${NC}"
        echo -e "${YELLOW}${ICON_INFO} Revisando los logs del frontend:${NC}"
        echo -e "${CYAN}╭───────────────────────────────────────────────────────────╮${NC}"
        docker-compose logs frontend
        echo -e "${CYAN}╰───────────────────────────────────────────────────────────╯${NC}"
        
        if [ $retry -lt $MAX_RETRIES ]; then
            echo -e "\n${YELLOW}${ICON_RETRY} Reintentando iniciar los contenedores...${NC}"
            docker-compose down > /dev/null 2>&1
            sleep 2
            start_containers $((retry + 1))
            return $?
        else
            echo -e "\n${RED}${ICON_WARNING} No se pudo iniciar el frontend después de $((MAX_RETRIES+1)) intentos.${NC}"
            echo -e "${YELLOW}${ICON_INFO} El servicio API y MySQL están disponibles, pero el frontend podría no funcionar correctamente.${NC}"
            return 1
        }
    fi
    
    return 0
}

# Iniciar los contenedores
start_containers
success=$?

# Resumen final con URLs y estado
echo -e "\n${CYAN}${BOLD}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}${BOLD}║                    RESUMEN DE SERVICIOS                    ║${NC}"
echo -e "${CYAN}${BOLD}╠════════════════════════════════════════════════════════════╣${NC}"

if [ $success -eq 0 ]; then
    echo -e "${CYAN}${BOLD}║${NC} ${GREEN}${BOLD}${ICON_CHECK} ¡Todos los servicios iniciados correctamente!${NC}        ${CYAN}${BOLD}║${NC}"
else
    echo -e "${CYAN}${BOLD}║${NC} ${YELLOW}${BOLD}${ICON_WARNING} Servicios iniciados parcialmente${NC}                  ${CYAN}${BOLD}║${NC}"
fi

echo -e "${CYAN}${BOLD}╠════════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}${BOLD}║${NC} ${ICON_WEB} ${BOLD}Frontend:${NC}        http://localhost:3001           ${CYAN}${BOLD}║${NC}"
echo -e "${CYAN}${BOLD}║${NC} ${ICON_WEB} ${BOLD}Backend API:${NC}     http://localhost:3000           ${CYAN}${BOLD}║${NC}"
echo -e "${CYAN}${BOLD}║${NC} ${ICON_DOCS} ${BOLD}Documentación:${NC}  http://localhost:3000/api-docs  ${CYAN}${BOLD}║${NC}"
echo -e "${CYAN}${BOLD}║${NC} ${ICON_DATABASE} ${BOLD}MySQL:${NC}          localhost:3306              ${CYAN}${BOLD}║${NC}"
echo -e "${CYAN}${BOLD}╚════════════════════════════════════════════════════════════╝${NC}"

echo -e "\n${GREEN}${BOLD}¡Listo para comenzar a trabajar con AidGuide!${NC}"
echo -e "${YELLOW}${ICON_INFO} Para detener los servicios, ejecute: ${CYAN}docker-compose down${NC} en el directorio del backend\n"

# Volvemos al directorio original
cd "$SCRIPT_DIR" || exit 