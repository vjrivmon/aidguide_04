#!/bin/bash

# Script para iniciar el proyecto completo AidGuide 04
# Autor: AidBot
# Fecha: 2024

# Variables de configuración
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_PATH="$SCRIPT_DIR/aidguide_04_ws"
PROJECT_NAME="aidguide_04"

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
ICON_ROS="🤖"
ICON_LOADING="⏳"
ICON_RETRY="🔄"
ICON_WEB="🌐"
ICON_CODE="💻"
ICON_DOCS="📄"

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
    echo -e "${CYAN}${BOLD}║                 ${ICON_ROCKET} AIDGUIDE PROYECTO ${ICON_ROCKET}                ║${NC}"
    echo -e "${CYAN}${BOLD}╚════════════════════════════════════════════════════════════╝${NC}\n"
    echo -e "${BLUE}${BOLD}Iniciando todos los componentes del proyecto AidGuide 04...${NC}\n"
}

# Función para verificar si un archivo existe
verify_file() {
    local path=$1
    local name=$2
    echo -ne "${YELLOW}${ICON_INFO} Verificando archivo ${BOLD}$name${NC}... "
    if [ ! -f "$path" ]; then
        echo -e "${RED}${ICON_ERROR} No encontrado${NC}"
        echo -e "${YELLOW}Buscando en: $(pwd)${NC}"
        echo -e "${YELLOW}Archivos disponibles:${NC}"
        ls -la "$SCRIPT_DIR"
        return 1
    fi
    echo -e "${GREEN}${ICON_CHECK} Encontrado${NC}"
    return 0
}

# Función para verificar si un servicio está instalado
verify_service() {
    local service=$1
    local package=$2
    
    echo -ne "${YELLOW}${ICON_INFO} Verificando $service... ${NC}"
    if ! command -v $service &> /dev/null; then
        echo -e "${YELLOW}${ICON_WARNING} No instalado${NC}"
        echo -e "${YELLOW}${ICON_INFO} Se requiere $package para continuar.${NC}"
        
        # Preguntar si desea instalarlo
        echo -e "${CYAN}¿Desea instalar $package ahora? (s/n)${NC}"
        read -r respuesta
        if [[ "$respuesta" =~ ^[Ss]$ ]]; then
            echo -e "${CYAN}${ICON_LOADING} Instalando $package...${NC}"
            sudo apt update && sudo apt install -y $package
            if ! command -v $service &> /dev/null; then
                echo -e "${RED}${ICON_ERROR} Error al instalar $package${NC}"
                return 1
            else
                echo -e "${GREEN}${ICON_CHECK} $package instalado correctamente${NC}"
            fi
        else
            echo -e "${RED}${ICON_ERROR} Se requiere $package para continuar.${NC}"
            return 1
        fi
    else
        echo -e "${GREEN}${ICON_CHECK} Instalado${NC}"
    fi
    return 0
}

# Función para verificar si Docker está instalado y en ejecución
check_docker() {
    echo -ne "${YELLOW}${ICON_INFO} Verificando Docker... ${NC}"
    
    # Comprobar si Docker está instalado
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}${ICON_ERROR} No instalado${NC}"
        echo -e "${YELLOW}${ICON_INFO} Docker no está instalado. El backend no se ejecutará.${NC}"
        echo -e "${YELLOW}${ICON_INFO} Para instalar Docker, visita: https://docs.docker.com/engine/install/${NC}"
        return 1
    fi
    
    # Verificar si el usuario pertenece al grupo docker
    if ! groups | grep -q "\\bdocker\\b"; then
        echo -e "${YELLOW}${ICON_WARNING} Tu usuario no pertenece al grupo docker${NC}"
        echo -e "${YELLOW}${ICON_INFO} Esto requiere usar 'sudo' para todos los comandos docker.${NC}"
        echo -e "${YELLOW}${ICON_INFO} Para evitar esto, puedes añadir tu usuario al grupo docker:${NC}"
        echo -e "${CYAN}   sudo usermod -aG docker $USER${NC}"
        echo -e "${YELLOW}${ICON_INFO} Después deberás cerrar sesión y volver a iniciarla para que los cambios tengan efecto.${NC}"
        echo -e "${YELLOW}${ICON_INFO} ¿Deseas añadir tu usuario al grupo docker ahora? (s/n)${NC}"
        read -r respuesta
        if [[ "$respuesta" =~ ^[Ss]$ ]]; then
            echo -e "${CYAN}${ICON_LOADING} Añadiendo usuario al grupo docker...${NC}"
            if sudo usermod -aG docker $USER; then
                echo -e "${GREEN}${ICON_CHECK} Usuario añadido al grupo docker${NC}"
                echo -e "${YELLOW}${ICON_INFO} Por favor, cierra sesión y vuelve a iniciarla para que los cambios tengan efecto.${NC}"
                echo -e "${YELLOW}${ICON_INFO} Mientras tanto, seguiremos usando 'sudo' para los comandos docker.${NC}"
            else
                echo -e "${RED}${ICON_ERROR} Error al añadir usuario al grupo docker${NC}"
            fi
        fi
    else
        echo -e "${GREEN}${ICON_CHECK} Usuario en grupo docker${NC}"
    fi
    
    # Comprobar si Docker está en ejecución
    if ! sudo docker info &> /dev/null; then
        echo -e "${YELLOW}${ICON_WARNING} Instalado pero no en ejecución${NC}"
        echo -e "${YELLOW}${ICON_INFO} Docker no está en ejecución. ¿Deseas iniciarlo? (s/n)${NC}"
        read -r respuesta
        if [[ "$respuesta" =~ ^[Ss]$ ]]; then
            echo -e "${CYAN}${ICON_LOADING} Intentando iniciar Docker...${NC}"
            
            # Intento 1: systemd
            if command -v systemctl &> /dev/null && sudo systemctl start docker; then
                echo -e "${CYAN}${ICON_LOADING} Esperando que Docker se inicie (systemctl)...${NC}"
                sleep 8
            # Intento 2: service
            elif command -v service &> /dev/null && sudo service docker start; then
                echo -e "${CYAN}${ICON_LOADING} Esperando que Docker se inicie (service)...${NC}"
                sleep 8
            # Intento 3: init.d script
            elif [ -f "/etc/init.d/docker" ] && sudo /etc/init.d/docker start; then
                echo -e "${CYAN}${ICON_LOADING} Esperando que Docker se inicie (init.d)...${NC}"
                sleep 8
            # Intento 4: dockerd
            elif command -v dockerd &> /dev/null; then
                echo -e "${CYAN}${ICON_LOADING} Intentando iniciar Docker daemon directamente...${NC}"
                sudo dockerd > /dev/null 2>&1 &
                sleep 10
            else
                echo -e "${YELLOW}${ICON_WARNING} No se pudieron detectar métodos para iniciar Docker${NC}"
                echo -e "${YELLOW}${ICON_INFO} Intenta iniciar Docker manualmente con uno de estos comandos:${NC}"
                echo -e "${CYAN}   sudo systemctl start docker${NC}"
                echo -e "${CYAN}   sudo service docker start${NC}"
                echo -e "${CYAN}   sudo /etc/init.d/docker start${NC}"
                echo -e "${YELLOW}${ICON_INFO} El backend no se ejecutará.${NC}"
                return 1
            fi
            
            # Verificar nuevamente
            if ! sudo docker info &> /dev/null; then
                echo -e "${RED}${ICON_ERROR} No se pudo iniciar Docker${NC}"
                echo -e "${YELLOW}${ICON_INFO} El backend no se ejecutará.${NC}"
                echo -e "${YELLOW}${ICON_INFO} Pero aún puedes acceder a los recursos del backend cuando lo inicies:${NC}"
                echo -e "${CYAN}   - Backend API:     http://localhost:3000${NC}"
                echo -e "${CYAN}   - Documentación:   http://localhost:3000/api-docs${NC}"
                return 1
            else
                echo -e "${GREEN}${ICON_CHECK} Docker iniciado correctamente${NC}"
                return 0
            fi
        else
            echo -e "${YELLOW}${ICON_INFO} El backend no se ejecutará sin Docker.${NC}"
            echo -e "${YELLOW}${ICON_INFO} Cuando inicies Docker manualmente, podrás acceder a:${NC}"
            echo -e "${CYAN}   - Backend API:     http://localhost:3000${NC}"
            echo -e "${CYAN}   - Documentación:   http://localhost:3000/api-docs${NC}"
            return 1
        fi
    fi
    
    echo -e "${GREEN}${ICON_CHECK} En ejecución${NC}"
    return 0
}

# Función para iniciar una aplicación en una nueva terminal
start_app_terminal() {
    local title=$1
    local script=$2
    local icon=$3
    local color=$4
    
    echo -e "${color}${icon} Iniciando $title...${NC}"
    
    # Intentar diferentes terminales disponibles
    gnome-terminal --title="$title" -- bash -c "cd \"$SCRIPT_DIR\" && bash \"$script\"; read -p 'Presiona Enter para cerrar esta terminal...'" 2>/dev/null || \
    xterm -T "$title" -e "cd \"$SCRIPT_DIR\" && bash \"$script\"; read -p 'Presiona Enter para cerrar esta terminal...'" 2>/dev/null || \
    konsole --new-tab -p tabtitle="$title" -e "cd \"$SCRIPT_DIR\" && bash \"$script\"; read -p 'Presiona Enter para cerrar esta terminal...'" 2>/dev/null || \
    {
        echo -e "${RED}${ICON_ERROR} No se pudo iniciar una nueva terminal para $title${NC}"
        echo -e "${YELLOW}${ICON_INFO} Ejecutando directamente:${NC}"
        bash "$SCRIPT_DIR/$script"
        return 1
    }
    
    return 0
}

# Función para verificar si existe un archivo y ejecutarlo
run_if_exists() {
    local script_path=$1
    local title=$2
    local icon=$3
    local color=$4
    
    if [ -f "$script_path" ]; then
        start_app_terminal "$title" "$script_path" "$icon" "$color"
        return $?
    else
        echo -e "${RED}${ICON_ERROR} El script $script_path no existe${NC}"
        return 1
    fi
}

# Función para sincronizar el docker-compose con el equipo
sync_docker_compose() {
    local source_path="$SCRIPT_DIR/aidguide_04_ws/src/aidguide_04_backend/docker-compose.yml"
    
    echo -e "${CYAN}${BOLD}▶ SINCRONIZANDO DOCKER-COMPOSE${NC}"
    echo -e "${YELLOW}${ICON_INFO} Verificando archivo docker-compose.yml...${NC}"
    
    if [ ! -f "$source_path" ]; then
        echo -e "${RED}${ICON_ERROR} No se encontró el archivo docker-compose.yml${NC}"
        return 1
    fi
    
    echo -e "${GREEN}${ICON_CHECK} Archivo docker-compose.yml encontrado${NC}"
    
    # Verificar si existe un directorio compartido para el equipo
    local team_dir="$SCRIPT_DIR/team_shared"
    if [ ! -d "$team_dir" ]; then
        echo -e "${YELLOW}${ICON_INFO} Creando directorio compartido para el equipo...${NC}"
        mkdir -p "$team_dir"
    fi
    
    # Copiar el archivo docker-compose.yml al directorio compartido
    echo -e "${YELLOW}${ICON_INFO} Copiando la última versión de docker-compose.yml para el equipo...${NC}"
    cp "$source_path" "$team_dir/docker-compose.latest.yml"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}${ICON_CHECK} Docker-compose sincronizado correctamente${NC}"
        echo -e "${CYAN}${ICON_INFO} Ubicación: $team_dir/docker-compose.latest.yml${NC}"
    else
        echo -e "${RED}${ICON_ERROR} Error al sincronizar docker-compose${NC}"
        return 1
    fi
    
    return 0
}

# Función principal
main() {
    # Mostrar encabezado
    show_header
    
    # Sincronizar docker-compose
    sync_docker_compose
    echo
    
    # Verificar dependencias
    echo -e "${CYAN}${BOLD}▶ VERIFICANDO DEPENDENCIAS${NC}"
    verify_service "bc" "bc" || exit 1
    
    # Verificar Docker para el backend
    DOCKER_OK=false
    if check_docker; then
        DOCKER_OK=true
    fi
    echo
    
    # Verificar archivos necesarios
    echo -e "${CYAN}${BOLD}▶ VERIFICANDO SCRIPTS${NC}"
    verify_file "$SCRIPT_DIR/start-ros2-gazebo.sh" "ROS2 & Gazebo" || exit 1
    verify_file "$SCRIPT_DIR/start-frontend.sh" "Frontend" || exit 1
    echo
    
    # Verificar permisos de ejecución
    echo -e "${CYAN}${BOLD}▶ VERIFICANDO PERMISOS${NC}"
    echo -e "${YELLOW}${ICON_INFO} Asegurando permisos de ejecución...${NC}"
    chmod +x "$SCRIPT_DIR/start-ros2-gazebo.sh"
    chmod +x "$SCRIPT_DIR/start-frontend.sh"
    echo -e "${GREEN}${ICON_CHECK} Permisos establecidos${NC}"
    echo
    
    # Preparar backend (si existe)
    echo -e "${CYAN}${BOLD}▶ VERIFICANDO BACKEND${NC}"
    if [ -f "$SCRIPT_DIR/aidguide_04_ws/start-services.sh" ]; then
        echo -e "${GREEN}${ICON_CHECK} Backend encontrado${NC}"
        chmod +x "$SCRIPT_DIR/aidguide_04_ws/start-services.sh"
    else
        echo -e "${YELLOW}${ICON_WARNING} Script de backend no encontrado (opcional)${NC}"
    fi
    echo
    
    # Verificar paquete de monitorización del robot
    echo -e "${CYAN}${BOLD}▶ VERIFICANDO MONITORIZACIÓN DEL ROBOT${NC}"
    ROS_MONITORING_OK=false
    if [ -d "$WORKSPACE_PATH/src/aidguide_04_robot_monitoring" ]; then
        echo -e "${GREEN}${ICON_CHECK} Paquete de monitorización de robot encontrado${NC}"
        ROS_MONITORING_OK=true
    else
        echo -e "${YELLOW}${ICON_WARNING} Paquete de monitorización de robot no encontrado${NC}"
    fi
    echo
    
    # Iniciar componentes
    echo -e "${CYAN}${BOLD}▶ INICIANDO COMPONENTES${NC}"
    
    # 1. Iniciar Backend (si existe y Docker está disponible)
    if [ -f "$SCRIPT_DIR/aidguide_04_ws/start-services.sh" ] && [ "$DOCKER_OK" = true ]; then
        echo -e "${BLUE}${BOLD}1. Iniciando Backend...${NC}"
        run_if_exists "$SCRIPT_DIR/aidguide_04_ws/start-services.sh" "AidGuide Backend" "${ICON_CODE}" "${BLUE}"
        show_progress 2 "${ICON_LOADING} Esperando que el backend se inicialice..."
    else
        if [ -f "$SCRIPT_DIR/aidguide_04_ws/start-services.sh" ]; then
            echo -e "${YELLOW}${ICON_WARNING} Backend no iniciado - Docker no disponible${NC}"
        fi
    fi
    
    # 2. Iniciar ROS2 y Gazebo
    echo -e "${MAGENTA}${BOLD}2. Iniciando ROS2 y Gazebo...${NC}"
    run_if_exists "$SCRIPT_DIR/start-ros2-gazebo.sh" "ROS2 & Gazebo" "${ICON_ROS}" "${MAGENTA}"
    show_progress 2 "${ICON_LOADING} Esperando que ROS2 se inicialice..."
    
    # 3. Iniciar sistema de monitorización del robot
    if [ "$ROS_MONITORING_OK" = true ]; then
        echo -e "${YELLOW}${BOLD}3. Iniciando Sistema de Monitorización del Robot...${NC}"
        
        # Crear un script temporal para iniciar los monitores
        MONITOR_SCRIPT="/tmp/start_robot_monitoring.sh"
        cat > "$MONITOR_SCRIPT" << EOL
#!/bin/bash
cd "$WORKSPACE_PATH"
source /opt/ros/galactic/setup.bash

# Compilar el paquete de monitorización si es necesario
echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${YELLOW}🤖 MONITORIZACIÓN DEL ROBOT                             ${CYAN}  ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"

echo -e "${YELLOW}🔧 Compilando el paquete de monitorización...${NC}"
colcon build --packages-select aidguide_04_robot_monitoring

# Cargar el entorno de trabajo
source install/setup.bash

# Comprobar si existe el archivo de lanzamiento de monitorización
if [ -f "$WORKSPACE_PATH/src/aidguide_04_robot_monitoring/launch/monitoring.launch.py" ]; then
    echo -e "${YELLOW}🔍 Iniciando monitores con el archivo de lanzamiento...${NC}"
    ros2 launch aidguide_04_robot_monitoring monitoring.launch.py
else
    echo -e "${YELLOW}🔍 Iniciando monitores individualmente...${NC}"
    
    # Iniciamos cada nodo de monitorización
    echo -e "${GREEN}✅ Iniciando monitor de batería${NC}"
    ros2 run aidguide_04_robot_monitoring battery_monitor &
    BATTERY_PID=\$!
    
    echo -e "${GREEN}✅ Iniciando monitor de hardware${NC}"
    ros2 run aidguide_04_robot_monitoring hardware_monitor &
    HARDWARE_PID=\$!
    
    echo -e "${GREEN}✅ Iniciando monitor de temperatura${NC}"
    ros2 run aidguide_04_robot_monitoring temperature_monitor &
    TEMP_PID=\$!
    
    echo -e "${GREEN}✅ Iniciando monitor de logs${NC}"
    ros2 run aidguide_04_robot_monitoring log_monitor &
    LOG_PID=\$!
    
    echo -e "${GREEN}✅ Iniciando panel de monitorización${NC}"
    ros2 run aidguide_04_robot_monitoring monitoring_dashboard
    
    # Detener todos los procesos al salir
    kill \$BATTERY_PID \$HARDWARE_PID \$TEMP_PID \$LOG_PID
fi

echo -e "${YELLOW}Monitorización finalizada${NC}"
read -p "Presiona Enter para cerrar esta terminal..."
EOL
        
        chmod +x "$MONITOR_SCRIPT"
        start_app_terminal "Monitorización del Robot" "$MONITOR_SCRIPT" "${ICON_ROS}" "${YELLOW}"
        show_progress 2 "${ICON_LOADING} Esperando que los monitores se inicialicen..."
    fi
    
    # 4. Iniciar Frontend
    echo -e "${GREEN}${BOLD}4. Iniciando Frontend...${NC}"
    run_if_exists "$SCRIPT_DIR/start-frontend.sh" "AidGuide Frontend" "${ICON_WEB}" "${GREEN}"
    
    # Resumen final
    echo -e "\n${CYAN}${BOLD}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}${BOLD}║                    RESUMEN DEL PROYECTO                    ║${NC}"
    echo -e "${CYAN}${BOLD}╠════════════════════════════════════════════════════════════╣${NC}"
    
    if [ "$DOCKER_OK" = true ] && [ -f "$SCRIPT_DIR/aidguide_04_ws/start-services.sh" ]; then
        echo -e "${CYAN}${BOLD}║${NC} ${GREEN}${BOLD}${ICON_CHECK} ¡Todos los componentes iniciados correctamente!${NC}    ${CYAN}${BOLD}║${NC}"
    else
        echo -e "${CYAN}${BOLD}║${NC} ${YELLOW}${BOLD}${ICON_WARNING} Componentes iniciados parcialmente${NC}             ${CYAN}${BOLD}║${NC}"
        if [ "$DOCKER_OK" = false ] && [ -f "$SCRIPT_DIR/aidguide_04_ws/start-services.sh" ]; then
            echo -e "${CYAN}${BOLD}║${NC} ${YELLOW}${BOLD}${ICON_WARNING} Backend no disponible - Docker no en ejecución${NC}  ${CYAN}${BOLD}║${NC}"
        fi
    fi
    
    if [ "$ROS_MONITORING_OK" = true ]; then
        echo -e "${CYAN}${BOLD}║${NC} ${GREEN}${BOLD}${ICON_CHECK} Sistema de monitorización del robot activo${NC}      ${CYAN}${BOLD}║${NC}"
    fi
    
    echo -e "${CYAN}${BOLD}╠════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}${BOLD}║${NC} ${ICON_ROS} ${BOLD}ROS2 & Gazebo:${NC}  Ejecutándose en terminales separadas ${CYAN}${BOLD}║${NC}"
    echo -e "${CYAN}${BOLD}║${NC} ${ICON_WEB} ${BOLD}Frontend:${NC}      http://localhost:3334              ${CYAN}${BOLD}║${NC}"
    # Mostrar información del backend independientemente de si Docker está en ejecución
    echo -e "${CYAN}${BOLD}║${NC} ${ICON_CODE} ${BOLD}Backend API:${NC}    http://localhost:3333/api       ${CYAN}${BOLD}║${NC}"
    echo -e "${CYAN}${BOLD}║${NC} ${ICON_DOCS} ${BOLD}Documentación:${NC} http://localhost:3333/api-docs   ${CYAN}${BOLD}║${NC}"
    if [ "$DOCKER_OK" = false ] && [ -f "$SCRIPT_DIR/aidguide_04_ws/start-services.sh" ]; then
        echo -e "${CYAN}${BOLD}║${NC} ${YELLOW}${BOLD}${ICON_WARNING} NOTA: El backend estará disponible cuando${NC}      ${CYAN}${BOLD}║${NC}"
        echo -e "${CYAN}${BOLD}║${NC} ${YELLOW}${BOLD}${ICON_WARNING} inicies Docker y ejecutes: ./aidguide_04_ws/start-services.sh${NC} ${CYAN}${BOLD}║${NC}"
    fi
    echo -e "${CYAN}${BOLD}╚════════════════════════════════════════════════════════════╝${NC}"
    echo -e "\n${GREEN}${BOLD}¡AidGuide 04 está ejecutándose ahora!${NC}"
    echo -e "${YELLOW}${ICON_INFO} IMPORTANTE: No cierres esta terminal hasta que hayas terminado de usar la aplicación.${NC}\n"
    
    # Mantener este script en ejecución para que sea fácil cerrar todo
    echo -e "${CYAN}Presiona Ctrl+C para detener todos los servicios...${NC}"
    trap "echo -e '${YELLOW}Deteniendo servicios...${NC}'; exit" INT
    while true; do
        sleep 1
    done
}

# Ejecutar la función principal
main 