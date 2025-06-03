#!/bin/bash

# Script para iniciar todos los servicios de AidGuide (Frontend y Backend)
# Autor: Vicente
# Fecha: 2024

# Variables de configuración
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Corrección de rutas - considerando que los directorios podrían estar en el nivel superior
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BACKEND_PATH="$PROJECT_ROOT/aidguide_04_backend"
FRONTEND_PATH="$PROJECT_ROOT/aidguide_04_web"
PROJECT_NAME="aidguide_04"
MAX_RETRIES=2
# Flag para controlar si podemos iniciar Docker
CAN_START_DOCKER=true

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
    local optional=${3:-false}
    
    echo -ne "${YELLOW}${ICON_INFO} Verificando directorio ${BOLD}$name${NC}... "
    if [ ! -d "$path" ]; then
        echo -e "${RED}${ICON_ERROR} No encontrado${NC}"
        
        if [ "$optional" = false ]; then
            echo -e "${YELLOW}Directorio actual: $(pwd)${NC}"
            echo -e "${YELLOW}Directorios disponibles:${NC}"
            
            # Mostrar directorio raíz del proyecto
            echo -e "${YELLOW}Contenido de $PROJECT_ROOT:${NC}"
            ls -la "$PROJECT_ROOT" 2>/dev/null || echo -e "${RED}No se puede acceder a $PROJECT_ROOT${NC}"
            
            # También mostrar el directorio scripts
            echo -e "${YELLOW}Contenido de $SCRIPT_DIR:${NC}"
            ls -la "$SCRIPT_DIR"
            
            if [ "$name" = "Backend" ]; then
                echo -e "${YELLOW}${ICON_WARNING} No se encontró el directorio Backend. No se podrán iniciar los servicios Docker.${NC}"
                CAN_START_DOCKER=false
            fi
            
            return 1
        else
            echo -e "${YELLOW}${ICON_WARNING} No encontrado (opcional)${NC}"
            return 0
        fi
    fi
    
    echo -e "${GREEN}${ICON_CHECK} Encontrado${NC}"
    return 0
}

# Mostrar el encabezado
show_header

# Verificar si los directorios existen
echo -e "${CYAN}${BOLD}▶ VERIFICANDO DIRECTORIOS${NC}"
# Marcamos como opcional (true) para que continúe aunque no exista
verify_directory "$BACKEND_PATH" "Backend" true
verify_directory "$FRONTEND_PATH" "Frontend" true
echo

# Seccion Docker - Solo se ejecuta si CAN_START_DOCKER es true
if [ "$CAN_START_DOCKER" = true ] && [ -d "$BACKEND_PATH" ]; then
    # Navegamos al directorio del backend donde está el docker-compose.yml
    echo -e "${CYAN}${BOLD}▶ PREPARANDO ENTORNO${NC}"
    echo -e "${YELLOW}${ICON_FOLDER} Navegando a ${BOLD}$BACKEND_PATH${NC}"
    cd "$BACKEND_PATH" || { 
        echo -e "${RED}${ICON_ERROR} Error: No se pudo acceder al directorio $BACKEND_PATH${NC}"
        CAN_START_DOCKER=false
    }

    if [ "$CAN_START_DOCKER" = true ]; then
        # Comprobamos si Docker está en ejecución
        echo -ne "${YELLOW}${ICON_INFO} Comprobando estado de Docker... ${NC}"
        if ! sudo docker info > /dev/null 2>&1; then
            echo -e "${RED}${ICON_ERROR} No en ejecución${NC}"
            echo -e "${YELLOW}${ICON_INFO} Intentando iniciar Docker...${NC}"
            
            # Intentar diferentes métodos para iniciar Docker
            DOCKER_STARTED=false
            
            # Método 1: systemctl
            if sudo systemctl start docker; then
                echo -e "${CYAN}${ICON_LOADING} Esperando que Docker se inicie (systemctl)...${NC}"
                sleep 8
                if sudo docker info > /dev/null 2>&1; then
                    DOCKER_STARTED=true
                    echo -e "${GREEN}${ICON_CHECK} Docker iniciado correctamente${NC}"
                fi
            fi
            
            # Método 2: service
            if [ "$DOCKER_STARTED" = false ] && sudo service docker start; then
                echo -e "${CYAN}${ICON_LOADING} Esperando que Docker se inicie (service)...${NC}"
                sleep 8
                if sudo docker info > /dev/null 2>&1; then
                    DOCKER_STARTED=true
                    echo -e "${GREEN}${ICON_CHECK} Docker iniciado correctamente${NC}"
                fi
            fi
            
            # Si no se pudo iniciar Docker
            if [ "$DOCKER_STARTED" = false ]; then
                echo -e "${RED}${BOLD}Docker no se pudo iniciar automáticamente.${NC}"
                echo -e "${YELLOW}${ICON_INFO} Para iniciar Docker manualmente, prueba uno de estos comandos:${NC}"
                echo -e "${CYAN}   1. sudo systemctl start docker${NC}"
                echo -e "${CYAN}   2. sudo service docker start${NC}"
                echo -e "${CYAN}   3. sudo /etc/init.d/docker start${NC}"
                
                # Preguntar si desea continuar sin Docker
                echo -e "${YELLOW}${ICON_INFO} ¿Deseas continuar sin Docker? (s/n)${NC}"
                read -r respuesta
                if [[ "$respuesta" =~ ^[Ss]$ ]]; then
                    echo -e "${YELLOW}${ICON_WARNING} Continuando sin Docker. Solo se iniciará Ollama.${NC}"
                    CAN_START_DOCKER=false
                else
                    exit 1
                fi
            fi
        else
            echo -e "${GREEN}${ICON_CHECK} En ejecución${NC}"
        fi

        # Verificar si existe el archivo docker-compose.yml
        if [ "$CAN_START_DOCKER" = true ]; then
            echo -ne "${YELLOW}${ICON_INFO} Verificando archivo docker-compose.yml... ${NC}"
            if [ ! -f "docker-compose.yml" ]; then
                echo -e "${RED}${ICON_ERROR} No encontrado${NC}"
                echo -e "${RED}${BOLD}Error: No se encontró el archivo docker-compose.yml en $(pwd)${NC}"
                CAN_START_DOCKER=false
            else
                echo -e "${GREEN}${ICON_CHECK} Encontrado${NC}"
            fi
            echo
        fi
    fi
else
    echo -e "${YELLOW}${ICON_WARNING} Omitiendo la verificación de Docker ya que el directorio Backend no está disponible.${NC}"
    CAN_START_DOCKER=false
fi

# Iniciar servicios Docker si es posible
success=0
if [ "$CAN_START_DOCKER" = true ]; then
    # Limpiar todos los contenedores relacionados con el proyecto
    echo -e "${CYAN}${BOLD}▶ PREPARANDO CONTENEDORES${NC}"
    echo -e "${YELLOW}${ICON_CLEANING} Limpiando contenedores existentes...${NC}"
    sudo docker-compose down > /dev/null 2>&1
    containers=$(sudo docker ps -a --filter "name=aidguide" --format "{{.Names}}")
    if [ ! -z "$containers" ]; then
        echo -e "  ${YELLOW}${ICON_INFO} Eliminando contenedores adicionales...${NC}"
        sudo docker rm -f $containers > /dev/null 2>&1
    fi
    show_progress 1 "${ICON_LOADING} Limpieza en progreso..."
    echo

    # Función para iniciar los contenedores y manejar reintentos
    start_containers() {
        local retry=${1:-0}
        local retry_num=$((retry+1))
        
        # Iniciamos los contenedores con docker-compose
        echo -e "${CYAN}${BOLD}▶ INICIANDO SERVICIOS (Intento ${retry_num}/${MAX_RETRIES+1})${NC}"
        echo -e "${YELLOW}${ICON_DOCKER} Creando y arrancando contenedores...${NC}"
        sudo docker-compose up -d --build
        
        # Mostramos una barra de progreso mientras los contenedores se inician
        show_progress 3 "${ICON_LOADING} Inicializando servicios..."
        
        # Verificamos el estado de los contenedores
        echo -e "\n${MAGENTA}${BOLD}${ICON_INFO} ESTADO DE LOS SERVICIOS:${NC}"
        echo -e "${CYAN}╭───────────────────────────────────────────────────────────╮${NC}"
        sudo docker-compose ps
        echo -e "${CYAN}╰───────────────────────────────────────────────────────────╯${NC}"
        
        # Verificar si el frontend está en ejecución
        if ! sudo docker ps --filter "name=aidguide_frontend" --format "{{.Names}}" | grep -q "aidguide_frontend"; then
            echo -e "\n${YELLOW}${ICON_WARNING} El contenedor del frontend no se inició correctamente.${NC}"
            echo -e "${YELLOW}${ICON_INFO} Revisando los logs del frontend:${NC}"
            echo -e "${CYAN}╭───────────────────────────────────────────────────────────╮${NC}"
            sudo docker-compose logs frontend
            echo -e "${CYAN}╰───────────────────────────────────────────────────────────╯${NC}"
            
            if [ $retry -lt $MAX_RETRIES ]; then
                echo -e "\n${YELLOW}${ICON_RETRY} Reintentando iniciar los contenedores...${NC}"
                sudo docker-compose down > /dev/null 2>&1
                sleep 2
                start_containers $((retry + 1))
                return $?
            else
                echo -e "\n${RED}${ICON_WARNING} No se pudo iniciar el frontend después de $((MAX_RETRIES+1)) intentos.${NC}"
                echo -e "${YELLOW}${ICON_INFO} El servicio API y MySQL están disponibles, pero el frontend podría no funcionar correctamente.${NC}"
                return 1
            fi
        fi
        
        return 0
    }

    # Iniciar los contenedores
    start_containers
    success=$?

    # Si no se pudieron iniciar los contenedores, ofrecer alternativas
    if [ $success -ne 0 ]; then
        echo -e "\n${YELLOW}${ICON_WARNING} No se pudieron iniciar todos los contenedores correctamente.${NC}"
        echo -e "${YELLOW}${ICON_INFO} ¿Deseas intentar iniciar solo la base de datos? (s/n)${NC}"
        read -r respuesta
        if [[ "$respuesta" =~ ^[Ss]$ ]]; then
            echo -e "${CYAN}${ICON_LOADING} Iniciando solo la base de datos...${NC}"
            sudo docker-compose up -d mysql
            show_progress 2 "${ICON_LOADING} Inicializando base de datos..."
        fi
    fi

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
    echo -e "${CYAN}${BOLD}║${NC} ${ICON_WEB} ${BOLD}Frontend:${NC}        http://localhost:3334           ${CYAN}${BOLD}║${NC}"
    echo -e "${CYAN}${BOLD}║${NC} ${ICON_WEB} ${BOLD}Backend API:${NC}     http://localhost:3333           ${CYAN}${BOLD}║${NC}"
    echo -e "${CYAN}${BOLD}║${NC} ${ICON_DOCS} ${BOLD}Documentación:${NC}  http://localhost:3333/api-docs  ${CYAN}${BOLD}║${NC}"
    echo -e "${CYAN}${BOLD}║${NC} ${ICON_DATABASE} ${BOLD}MySQL:${NC}          localhost:3306              ${CYAN}${BOLD}║${NC}"
    echo -e "${CYAN}${BOLD}╚════════════════════════════════════════════════════════════╝${NC}"

    echo -e "\n${GREEN}${BOLD}¡Servicios Docker de AidGuide iniciados!${NC}"
    echo -e "${YELLOW}${ICON_INFO} Para detener los servicios Docker, ejecute: ${CYAN}sudo docker-compose down${NC} en el directorio del backend\n"
else
    echo -e "\n${YELLOW}${ICON_WARNING} No se iniciaron los servicios Docker.${NC}"
    echo -e "${YELLOW}${ICON_INFO} Continuando solo con la inicialización de Ollama...${NC}\n"
fi

# Volvemos al directorio original (por si se cambió al intentar iniciar Docker)
cd "$SCRIPT_DIR" || exit 

# Iniciar servicio Ollama para el chatbot web
echo -e "${CYAN}${BOLD}▶ INICIANDO SERVICIO OLLAMA PARA CHATBOT${NC}"
echo -e "${YELLOW}${ICON_INFO} Verificando estado de Ollama...${NC}"

# Verificar si Ollama está instalado
if ! command -v ollama &> /dev/null; then
    echo -e "${RED}${ICON_ERROR} Ollama no está instalado en este sistema${NC}"
    echo -e "${YELLOW}${ICON_INFO} Instalando Ollama...${NC}"
    
    # Instalación de Ollama
    curl -fsSL https://ollama.com/install.sh | sh
    
    if [ $? -ne 0 ]; then
        echo -e "${RED}${ICON_ERROR} No se pudo instalar Ollama. El chatbot no estará disponible.${NC}"
    else
        echo -e "${GREEN}${ICON_CHECK} Ollama instalado correctamente${NC}"
    fi
else
    echo -e "${GREEN}${ICON_CHECK} Ollama ya está instalado${NC}"
fi

# Si Ollama está instalado, verificar modelo y iniciar el servidor
if command -v ollama &> /dev/null; then
    # Verificar si el modelo Mistral está disponible
    echo -e "${YELLOW}${ICON_INFO} Verificando modelo Mistral...${NC}"
    if ! ollama list | grep -q "mistral"; then
        echo -e "${YELLOW}${ICON_INFO} Descargando el modelo Mistral...${NC}"
        ollama pull mistral
        
        if [ $? -ne 0 ]; then
            echo -e "${RED}${ICON_ERROR} No se pudo descargar el modelo Mistral. El chatbot no funcionará correctamente.${NC}"
        else
            echo -e "${GREEN}${ICON_CHECK} Modelo Mistral descargado correctamente${NC}"
        fi
    else
        echo -e "${GREEN}${ICON_CHECK} El modelo Mistral ya está disponible${NC}"
    fi

    # Iniciar el servidor Ollama en segundo plano
    echo -e "${YELLOW}${ICON_INFO} Iniciando servidor Ollama...${NC}"
    ollama serve &
    OLLAMA_PID=$!
    sleep 2

    # Verificar si el servidor está en funcionamiento
    if ! curl -s http://localhost:11434/api/health &> /dev/null; then
        echo -e "${RED}${ICON_ERROR} No se pudo iniciar el servidor Ollama. El chatbot no estará disponible.${NC}"
    else
        echo -e "${GREEN}${ICON_CHECK} Servidor Ollama iniciado correctamente${NC}"
        echo -e "${CYAN}${ICON_INFO} API de Ollama disponible en: http://localhost:11434${NC}"
        
        # Añadir el servicio de Ollama al resumen
        echo -e "${CYAN}${BOLD}╔════════════════════════════════════════════════════════════╗${NC}"
        echo -e "${CYAN}${BOLD}║                  SERVICIO DE CHATBOT                      ║${NC}"
        echo -e "${CYAN}${BOLD}╠════════════════════════════════════════════════════════════╣${NC}"
        echo -e "${CYAN}${BOLD}║${NC} ${ICON_WEB} ${BOLD}API de Ollama:${NC}  http://localhost:11434          ${CYAN}${BOLD}║${NC}"
        echo -e "${CYAN}${BOLD}║${NC} ${BOLD}Modelo:${NC}         Mistral                          ${CYAN}${BOLD}║${NC}"
        echo -e "${CYAN}${BOLD}╚════════════════════════════════════════════════════════════╝${NC}"
        echo -e "${YELLOW}${ICON_INFO} El chatbot ahora puede utilizar el modelo Mistral${NC}"
        echo -e "${YELLOW}${ICON_WARNING} Este script debe mantenerse en ejecución para que Ollama siga funcionando${NC}"
    fi
fi

# Mensaje final y advertencias
echo -e "\n${GREEN}${BOLD}¡Todos los servicios solicitados de AidGuide están en funcionamiento!${NC}"
if [ "$CAN_START_DOCKER" = true ]; then
    echo -e "${YELLOW}${ICON_INFO} Para detener los servicios Docker, ejecute: ${CYAN}sudo docker-compose down${NC} en el directorio del backend"
fi
echo -e "${YELLOW}${ICON_INFO} Para detener el servicio Ollama, presione ${CYAN}Ctrl+C${NC} en esta terminal\n"

# Si Ollama está en ejecución, esperar a que termine para mantener el script activo
if [ -n "$OLLAMA_PID" ]; then
    echo -e "${CYAN}${ICON_INFO} Manteniendo servicios en ejecución...${NC}"
    echo -e "${YELLOW}${ICON_WARNING} No cierre esta terminal para mantener Ollama activo${NC}"
    wait $OLLAMA_PID
fi 