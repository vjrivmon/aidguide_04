#!/bin/bash

# Script de automatización para iniciar ROS2 y Gazebo
# Fecha: 2024
# Descripción: Este script automatiza el lanzamiento de todos los componentes necesarios para ROS2 y Gazebo

# Colores para los mensajes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Función para verificar si ROS2 está instalado y configurado
check_ros() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}❌ ROS2 no está instalado o no está en el PATH${NC}"
        echo -e "${YELLOW}📝 Por favor, asegúrate de que ROS2 está instalado y que has ejecutado:${NC}"
        echo -e "${YELLOW}   source /opt/ros/galactic/setup.bash${NC}"
        exit 1
    fi
}

# Determinar la ruta del workspace
get_workspace_path() {
    WORKSPACE_PATH="$(pwd)"
    if [[ "$WORKSPACE_PATH" != *"aidguide_04_ws"* ]]; then
        if [ -d "$HOME/aidguide_04/aidguide_04_ws" ]; then
            WORKSPACE_PATH="$HOME/aidguide_04/aidguide_04_ws"
        elif [ -d "$(pwd)/aidguide_04_ws" ]; then
            WORKSPACE_PATH="$(pwd)/aidguide_04_ws"
        else
            echo -e "${RED}❌ No se pudo encontrar el workspace de ROS2${NC}"
            exit 1
        fi
    fi
    echo "$WORKSPACE_PATH"
}

# Función para reemplazar rutas en archivos de configuración
replace_paths_in_files() {
    local old_base_path="$1"
    local new_base_path="$2"
    local workspace_path="$3"
    
    echo -e "${CYAN}🔧 Adaptando archivos de configuración${NC}"
    echo -e "${CYAN}   De: ${YELLOW}$old_base_path${NC}"
    echo -e "${CYAN}   A:  ${GREEN}$new_base_path${NC}"
    
    # Crear copias temporales de archivos antes de modificarlos
    echo -e "${BLUE}📝 Haciendo copias de seguridad de los archivos...${NC}"
    
    # Lista de archivos a modificar
    local files_to_modify=(
        "$workspace_path/src/aidguide_04_provide_map/map/aidguide_04_map.yaml"
        "$workspace_path/src/aidguide_04_nav_punto_a_punto/config/aidguide_04_map.yaml"
        "$workspace_path/src/aidguide_04_my_nav2_system/config/aidguide_config_robot.rviz"
    )
    
    # Crear copias de seguridad y modificar archivos
    for file in "${files_to_modify[@]}"; do
        if [ -f "$file" ]; then
            # Crear copia de seguridad
            cp "$file" "${file}.bak"
            
            # Escapar caracteres especiales para sed
            local escaped_old_path=$(echo "$old_base_path" | sed 's/\//\\\//g')
            local escaped_new_path=$(echo "$new_base_path" | sed 's/\//\\\//g')
            
            # Reemplazar las rutas en el archivo
            sed -i "s/$escaped_old_path/$escaped_new_path/g" "$file"
            echo -e "${GREEN}✅ Modificado: $file${NC}"
        else
            echo -e "${YELLOW}⚠️ Archivo no encontrado: $file${NC}"
        fi
    done
    
    echo -e "${GREEN}✅ Archivos modificados correctamente${NC}"
}

# Función para restaurar archivos originales
restore_original_files() {
    local workspace_path="$1"
    
    echo -e "${CYAN}🔄 Restaurando archivos originales...${NC}"
    
    # Lista de archivos a restaurar
    local files_to_restore=(
        "$workspace_path/src/aidguide_04_provide_map/map/aidguide_04_map.yaml"
        "$workspace_path/src/aidguide_04_nav_punto_a_punto/config/aidguide_04_map.yaml"
        "$workspace_path/src/aidguide_04_my_nav2_system/config/aidguide_config_robot.rviz"
    )
    
    # Restaurar archivos originales
    for file in "${files_to_restore[@]}"; do
        if [ -f "${file}.bak" ]; then
            mv "${file}.bak" "$file"
            echo -e "${GREEN}✅ Restaurado: $file${NC}"
        fi
    done
    
    echo -e "${GREEN}✅ Archivos originales restaurados${NC}"
}

# Extraer la ruta base actual de los archivos
extract_base_path() {
    local workspace_path="$1"
    local file="$workspace_path/src/aidguide_04_provide_map/map/aidguide_04_map.yaml"
    
    if [ -f "$file" ]; then
        # Extraer la ruta hasta aidguide_04_ws
        local current_path=$(grep "image:" "$file" | sed 's/image: //g')
        # Obtener la ruta base (hasta aidguide_04/)
        local base_path=$(echo "$current_path" | sed 's/\(\/home\/[^\/]*\/[^\/]*\/\).*/\1/')
        echo "$base_path"
    else
        echo "/home/visi02/aidguide_04/"
    fi
}

# Mostrar cabecera del script
clear
echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${YELLOW}🚀 AIDGUIDE 04 - SISTEMA DE NAVEGACIÓN ROS2 Y GAZEBO ${CYAN}     ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Verificar ROS2
echo -e "${YELLOW}🔍 Verificando instalación de ROS2...${NC}"
check_ros
echo -e "${GREEN}✅ ROS2 está correctamente instalado${NC}"

# Obtener la ruta del workspace
WORKSPACE_PATH=$(get_workspace_path)
echo -e "${YELLOW}📂 Workspace detectado en: ${CYAN}$WORKSPACE_PATH${NC}"
cd "$WORKSPACE_PATH" || exit 1

# Extraer la ruta base actual
CURRENT_BASE_PATH=$(extract_base_path "$WORKSPACE_PATH")
echo -e "${YELLOW}📂 Ruta base actual en archivos de configuración: ${CYAN}$CURRENT_BASE_PATH${NC}"

# Preguntar por la ruta base a utilizar
current_user=$(whoami)
default_base_path="$HOME/aidguide_04/"

echo -e "${CYAN}👤 Usuario actual: ${YELLOW}$current_user${NC}"
echo -e "${CYAN}❓ ¿Deseas configurar una ruta base personalizada?${NC}"
echo -e "${CYAN}   1) Usar ruta por defecto: ${GREEN}$default_base_path${NC}"
echo -e "${CYAN}   2) Especificar otra ruta${NC}"
read -p "> " path_option

if [ "$path_option" = "2" ]; then
    echo -e "${CYAN}📂 Introduce la ruta base donde se encuentra el proyecto (incluyendo /aidguide_04/ al final):${NC}"
    read -p "> " custom_base_path
    
    # Asegurar que la ruta termine con /
    if [[ "$custom_base_path" != */ ]]; then
        custom_base_path="$custom_base_path/"
    fi
    
    NEW_BASE_PATH="$custom_base_path"
else
    NEW_BASE_PATH="$default_base_path"
fi

echo -e "${GREEN}✅ Usando la ruta base: ${YELLOW}$NEW_BASE_PATH${NC}"

# Adaptar archivos con la nueva ruta base
replace_paths_in_files "$CURRENT_BASE_PATH" "$NEW_BASE_PATH" "$WORKSPACE_PATH"

# Configurar una trampa para restaurar archivos al salir
trap 'echo -e "${YELLOW}🔄 Limpiando archivos temporales...${NC}"; restore_original_files "$WORKSPACE_PATH"; exit' EXIT

# Función para iniciar terminal con comandos
start_terminal() {
    local title=$1
    local commands=$2
    local color=$3
    
    echo -e "${color}🖥️  Iniciando terminal: $title${NC}"
    
    # Intentar diferentes terminales disponibles
    gnome-terminal --title="$title" -- bash -c "$commands" 2>/dev/null || \
    xterm -T "$title" -e "bash -c \"$commands\"" 2>/dev/null || \
    konsole --new-tab -p tabtitle="$title" -e "bash -c \"$commands\"" 2>/dev/null || \
    {
        echo -e "${RED}❌ No se pudo iniciar una nueva terminal${NC}"
        echo -e "${YELLOW}📝 Ejecuta manualmente los siguientes comandos:${NC}"
        echo -e "${CYAN}$commands${NC}"
        return 1
    }
    
    return 0
}

# Terminal 1: Gazebo
TERMINAL1_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${BLUE}╔════════════════════════════════════════╗${NC}\" && \
    echo -e \"${BLUE}║  TERMINAL 1: GAZEBO                    ║${NC}\" && \
    echo -e \"${BLUE}╚════════════════════════════════════════╝${NC}\" && \
    echo -e \"${YELLOW}🔨 Compilando aidguide_04_world...${NC}\" && \
    colcon build --packages-select aidguide_04_world && \
    echo -e \"${YELLOW}🔄 Actualizando entorno...${NC}\" && \
    source install/setup.bash && \
    echo -e \"${CYAN}🌐 Lanzando Gazebo...${NC}\" && \
    ros2 launch aidguide_04_world world.launch.py; \
    read -p \"Presiona Enter para cerrar esta terminal...\""

# Terminal 2: Mapa
TERMINAL2_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${MAGENTA}╔════════════════════════════════════════╗${NC}\" && \
    echo -e \"${MAGENTA}║  TERMINAL 2: MAPA                      ║${NC}\" && \
    echo -e \"${MAGENTA}╚════════════════════════════════════════╝${NC}\" && \
    echo -e \"${YELLOW}🔨 Compilando aidguide_04_provide_map...${NC}\" && \
    colcon build --packages-select aidguide_04_provide_map && \
    echo -e \"${YELLOW}🔄 Actualizando entorno...${NC}\" && \
    source install/setup.bash && \
    echo -e \"${CYAN}🗺️  Lanzando mapa...${NC}\" && \
    ros2 launch aidguide_04_provide_map aidguide_04_provide_map.launch.py; \
    read -p \"Presiona Enter para cerrar esta terminal...\""

# Terminal 3: Localización
TERMINAL3_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${GREEN}╔════════════════════════════════════════╗${NC}\" && \
    echo -e \"${GREEN}║  TERMINAL 3: LOCALIZACIÓN              ║${NC}\" && \
    echo -e \"${GREEN}╚════════════════════════════════════════╝${NC}\" && \
    echo -e \"${YELLOW}🔨 Compilando aidguide_04_my_nav2_system...${NC}\" && \
    colcon build --packages-select aidguide_04_my_nav2_system && \
    echo -e \"${YELLOW}🔄 Actualizando entorno...${NC}\" && \
    source install/setup.bash && \
    echo -e \"${CYAN}📍 Lanzando localización...${NC}\" && \
    ros2 launch aidguide_04_my_nav2_system my_nav2_system.launch.py; \
    read -p \"Presiona Enter para cerrar esta terminal...\""

# Terminal 4: Cargar Mapa
TERMINAL4_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${YELLOW}╔════════════════════════════════════════╗${NC}\" && \
    echo -e \"${YELLOW}║  TERMINAL 4: CARGAR MAPA              ║${NC}\" && \
    echo -e \"${YELLOW}╚════════════════════════════════════════╝${NC}\" && \
    echo -e \"${CYAN}🗂️  Esperando que el servicio del mapa esté disponible...${NC}\" && \
    MAP_SERVICE='/map_server/load_map' && \
    MAX_RETRIES=30 && \
    retry_count=0 && \
    while [ \$retry_count -lt \$MAX_RETRIES ]; do \
        echo -e \"${YELLOW}🔍 Verificando servicio (\$((\$retry_count + 1))/\$MAX_RETRIES): \$MAP_SERVICE ${NC}\" && \
        if ros2 service list | grep -q \"\$MAP_SERVICE\"; then \
            echo -e \"${GREEN}✅ Servicio de mapa encontrado${NC}\" && \
            break; \
        fi && \
        retry_count=\$((\$retry_count + 1)) && \
        sleep 2; \
    done && \
    if [ \$retry_count -lt \$MAX_RETRIES ]; then \
        # Definimos posibles ubicaciones del mapa, incluyendo mapas de otros paquetes
        MAPS=( \
            \"$WORKSPACE_PATH/src/aidguide_04_provide_map/map/aidguide_04_map.yaml\" \
            \"$WORKSPACE_PATH/install/aidguide_04_provide_map/share/aidguide_04_provide_map/map/aidguide_04_map.yaml\" \
            \"$WORKSPACE_PATH/src/aidguide_04_my_nav2_system/config/aidguide_04_map.yaml\" \
            \"$WORKSPACE_PATH/src/aidguide_04_nav/config/aidguide_04_map.yaml\" \
        ) && \
        MAP_FOUND=false && \
        for MAP_FILE in \"\${MAPS[@]}\"; do \
            if [ -f \"\$MAP_FILE\" ]; then \
                echo -e \"${GREEN}✅ Archivo de mapa encontrado: \$MAP_FILE${NC}\" && \
                echo -e \"${CYAN}🗺️ Cargando mapa...${NC}\" && \
                ros2 service call \$MAP_SERVICE nav2_msgs/srv/LoadMap \"{map_url: '\$MAP_FILE'}\" && \
                MAP_FOUND=true && \
                break; \
            fi; \
        done && \
        if [ \"\$MAP_FOUND\" = false ]; then \
            echo -e \"${RED}❌ No se encontraron archivos de mapa en las ubicaciones esperadas${NC}\" && \
            echo -e \"${YELLOW}📝 Mapas disponibles:${NC}\" && \
            find $WORKSPACE_PATH -name \"*.yaml\" -type f | grep map | sort; \
            echo -e \"${YELLOW}📝 Por favor, especifica manualmente la ruta del mapa:${NC}\" && \
            read -p \"Ruta del mapa: \" MANUAL_MAP && \
            if [ -f \"\$MANUAL_MAP\" ]; then \
                echo -e \"${GREEN}✅ Cargando mapa: \$MANUAL_MAP${NC}\" && \
                ros2 service call \$MAP_SERVICE nav2_msgs/srv/LoadMap \"{map_url: '\$MANUAL_MAP'}\"; \
            else \
                echo -e \"${RED}❌ El archivo especificado no existe o no es accesible${NC}\"; \
            fi; \
        fi; \
    else \
        echo -e \"${RED}❌ Servicio de mapa no disponible después de \$MAX_RETRIES intentos${NC}\" && \
        echo -e \"${YELLOW}📝 Servicios disponibles:${NC}\" && \
        ros2 service list; \
    fi && \
    read -p \"Presiona Enter para cerrar esta terminal...\""

# Terminal 5: Navegación
TERMINAL5_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${RED}╔════════════════════════════════════════╗${NC}\" && \
    echo -e \"${RED}║  TERMINAL 5: NAVEGACIÓN               ║${NC}\" && \
    echo -e \"${RED}╚════════════════════════════════════════╝${NC}\" && \
    echo -e \"${CYAN}🚶 Lanzando navegación...${NC}\" && \
    ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true; \
    read -p \"Presiona Enter para cerrar esta terminal...\""

# Mostrar instrucciones de inicio
echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${YELLOW}🚀 INICIANDO SISTEMA DE NAVEGACIÓN                     ${CYAN}  ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${GREEN}Se abrirán 5 terminales con los diferentes componentes  ${CYAN}  ║${NC}"
echo -e "${CYAN}║  ${GREEN}Espera a que cada uno inicie antes de continuar         ${CYAN}  ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Iniciar las terminales en secuencia
echo -e "${CYAN}🚀 Iniciando terminales...${NC}"
sleep 1

# Ahora iniciamos primero el mapa y la localización
start_terminal "Terminal 1: Mapa" "$TERMINAL2_COMMANDS" "${MAGENTA}"
sleep 3

start_terminal "Terminal 2: Localización" "$TERMINAL3_COMMANDS" "${GREEN}" 
sleep 3

start_terminal "Terminal 3: Cargar Mapa" "$TERMINAL4_COMMANDS" "${YELLOW}"
sleep 3

start_terminal "Terminal 4: Navegación" "$TERMINAL5_COMMANDS" "${RED}"
sleep 3

# Gazebo es el último en iniciarse
start_terminal "Terminal 5: Gazebo" "$TERMINAL1_COMMANDS" "${BLUE}"

echo ""
echo -e "${GREEN}✅ Todos los componentes han sido iniciados${NC}"
echo -e "${YELLOW}📝 Para interactuar con la navegación, utiliza RViz y las herramientas proporcionadas${NC}"
echo ""
echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${GREEN}Sistema completo en funcionamiento                     ${CYAN}  ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}" 

# Nota: Los archivos originales se restaurarán automáticamente cuando se cierre el script
# gracias al comando trap configurado anteriormente 