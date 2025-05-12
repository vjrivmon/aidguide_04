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

# Inicializar ROS2 Galactic
echo -e "${YELLOW}🔍 Inicializando entorno ROS2 Galactic...${NC}"
if [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    echo -e "${GREEN}✅ Entorno ROS2 Galactic inicializado${NC}"
else
    echo -e "${RED}❌ No se encontró /opt/ros/galactic/setup.bash${NC}"
    echo -e "${YELLOW}📝 Verificando otras versiones de ROS2...${NC}"
    
    # Buscar otras instalaciones de ROS2
    if [ -d "/opt/ros" ]; then
        ros_versions=$(ls /opt/ros)
        if [ -n "$ros_versions" ]; then
            echo -e "${YELLOW}⚠️ Versiones de ROS2 encontradas: $ros_versions${NC}"
            read -p "¿Cuál versión deseas usar? " ros_version
            if [ -f "/opt/ros/$ros_version/setup.bash" ]; then
                source "/opt/ros/$ros_version/setup.bash"
                echo -e "${GREEN}✅ Entorno ROS2 $ros_version inicializado${NC}"
            else
                echo -e "${RED}❌ Versión no válida${NC}"
                exit 1
            fi
        else
            echo -e "${RED}❌ No se encontraron instalaciones de ROS2${NC}"
            exit 1
        fi
    else
        echo -e "${RED}❌ ROS2 no está instalado${NC}"
        echo -e "${YELLOW}📝 Por favor, instala ROS2 siguiendo las instrucciones en: https://docs.ros.org/en/galactic/Installation.html${NC}"
        exit 1
    fi
fi

# Función para verificar e instalar js-yaml para manipular archivos YAML
check_and_install_jsyaml() {
    echo -e "${YELLOW}🔍 Verificando dependencia: js-yaml${NC}"
    
    # Verificar si js-yaml está instalado
    if ! npm list --global js-yaml >/dev/null 2>&1 && ! npm list --depth=0 js-yaml >/dev/null 2>&1; then
        echo -e "${YELLOW}⚠️ La dependencia 'js-yaml' no está instalada${NC}"
        echo -e "${CYAN}❓ ¿Deseas instalar js-yaml? (s/n)${NC}"
        read -p "> " install_jsyaml
        
        if [ "$install_jsyaml" = "s" ] || [ "$install_jsyaml" = "S" ]; then
            echo -e "${YELLOW}📦 Instalando js-yaml...${NC}"
            if npm install --save js-yaml --legacy-peer-deps; then
                echo -e "${GREEN}✅ js-yaml instalado correctamente${NC}"
            else
                echo -e "${RED}❌ Error al instalar js-yaml${NC}"
                echo -e "${YELLOW}📝 Intenta instalarlo manualmente con: npm install --save js-yaml --legacy-peer-deps${NC}"
            fi
        else
            echo -e "${YELLOW}⚠️ Continuando sin instalar js-yaml. Algunas funciones pueden no estar disponibles.${NC}"
        fi
    else
        echo -e "${GREEN}✅ js-yaml ya está instalado${NC}"
    fi
}

# Función para verificar e instalar Node.js y npm
check_and_install_nodejs() {
    echo -e "${YELLOW}🔍 Verificando instalación de Node.js y npm...${NC}"
    
    # Verificar si Node.js está instalado
    if ! command -v node &> /dev/null; then
        echo -e "${YELLOW}⚠️ Node.js no está instalado${NC}"
        echo -e "${CYAN}❓ ¿Deseas instalar Node.js? (s/n)${NC}"
        read -p "> " install_nodejs
        
        if [ "$install_nodejs" = "s" ] || [ "$install_nodejs" = "S" ]; then
            echo -e "${YELLOW}📦 Instalando Node.js y npm...${NC}"
            if curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash - &&
               sudo apt-get install -y nodejs; then
                echo -e "${GREEN}✅ Node.js y npm instalados correctamente${NC}"
            else
                echo -e "${RED}❌ Error al instalar Node.js y npm${NC}"
                echo -e "${YELLOW}📝 Intenta instalarlo manualmente siguiendo las instrucciones en: https://nodejs.org/es/download/package-manager/${NC}"
                return 1
            fi
        else
            echo -e "${YELLOW}⚠️ Continuando sin instalar Node.js. La interfaz web y el chatbot no estarán disponibles.${NC}"
            return 1
        fi
    else
        echo -e "${GREEN}✅ Node.js está instalado ($(node -v))${NC}"
        
        # Verificar si npm está instalado
        if ! command -v npm &> /dev/null; then
            echo -e "${YELLOW}⚠️ npm no está instalado${NC}"
            echo -e "${CYAN}❓ ¿Deseas instalar npm? (s/n)${NC}"
            read -p "> " install_npm
            
            if [ "$install_npm" = "s" ] || [ "$install_npm" = "S" ]; then
                echo -e "${YELLOW}📦 Instalando npm...${NC}"
                if sudo apt-get install -y npm; then
                    echo -e "${GREEN}✅ npm instalado correctamente${NC}"
                else
                    echo -e "${RED}❌ Error al instalar npm${NC}"
                    echo -e "${YELLOW}📝 Intenta instalarlo manualmente con: sudo apt-get install -y npm${NC}"
                    return 1
                fi
            else
                echo -e "${YELLOW}⚠️ Continuando sin instalar npm. La interfaz web y el chatbot no estarán disponibles.${NC}"
                return 1
            fi
        else
            echo -e "${GREEN}✅ npm está instalado ($(npm -v))${NC}"
        fi
    fi
    
    return 0
}

# Función para verificar e instalar Ollama
check_and_install_ollama() {
    echo -e "${YELLOW}🔍 Verificando instalación de Ollama...${NC}"
    
    # Verificar si Ollama está instalado o si está corriendo como servicio
    if ! command -v ollama &> /dev/null && ! pgrep -f ollama > /dev/null; then
        echo -e "${YELLOW}⚠️ Ollama no está instalado o no está en ejecución${NC}"
        echo -e "${CYAN}❓ ¿Deseas instalar Ollama? (s/n)${NC}"
        read -p "> " install_ollama
        
        if [ "$install_ollama" = "s" ] || [ "$install_ollama" = "S" ]; then
            echo -e "${YELLOW}📦 Instalando Ollama...${NC}"
            if curl -fsSL https://ollama.com/install.sh | sh; then
                echo -e "${GREEN}✅ Ollama instalado correctamente${NC}"
                
                # Verificar disponibilidad del modelo mistral
                echo -e "${YELLOW}🔍 Verificando disponibilidad del modelo 'mistral'...${NC}"
                if ollama list | grep -q "mistral"; then
                    echo -e "${GREEN}✅ Modelo 'mistral' ya está disponible${NC}"
                else
                    echo -e "${YELLOW}⚠️ El modelo 'mistral' no está disponible${NC}"
                    echo -e "${CYAN}❓ ¿Deseas descargar el modelo 'mistral'? (s/n)${NC}"
                    read -p "> " download_mistral
                    
                    if [ "$download_mistral" = "s" ] || [ "$download_mistral" = "S" ]; then
                        echo -e "${YELLOW}📦 Descargando modelo 'mistral'...${NC}"
                        if ollama pull mistral; then
                            echo -e "${GREEN}✅ Modelo 'mistral' descargado correctamente${NC}"
                        else
                            echo -e "${RED}❌ Error al descargar el modelo 'mistral'${NC}"
                            echo -e "${YELLOW}📝 El chatbot no funcionará correctamente sin este modelo${NC}"
                        fi
                    else
                        echo -e "${YELLOW}⚠️ Continuando sin descargar el modelo 'mistral'. El chatbot no funcionará correctamente.${NC}"
                    fi
                fi
            else
                echo -e "${RED}❌ Error al instalar Ollama${NC}"
                echo -e "${YELLOW}📝 Intenta instalarlo manualmente siguiendo las instrucciones en: https://ollama.com/download${NC}"
                return 1
            fi
        else
            echo -e "${YELLOW}⚠️ Continuando sin instalar Ollama. El chatbot no estará disponible.${NC}"
            return 1
        fi
    else
        if pgrep -f ollama > /dev/null; then
            echo -e "${GREEN}✅ Ollama está en ejecución${NC}"
        else
            echo -e "${GREEN}✅ Ollama está instalado${NC}"
            # Intentar iniciar Ollama
            echo -e "${YELLOW}🔍 Iniciando servicio Ollama...${NC}"
            ollama serve &
            sleep 2
            if pgrep -f ollama > /dev/null; then
                echo -e "${GREEN}✅ Servicio Ollama iniciado correctamente${NC}"
            else
                echo -e "${RED}❌ No se pudo iniciar el servicio Ollama${NC}"
                echo -e "${YELLOW}📝 Intenta iniciarlo manualmente con: ollama serve${NC}"
                return 1
            fi
        fi
        
        # Verificar disponibilidad del modelo mistral
        echo -e "${YELLOW}🔍 Verificando disponibilidad del modelo 'mistral'...${NC}"
        if ollama list | grep -q "mistral"; then
            echo -e "${GREEN}✅ Modelo 'mistral' está disponible${NC}"
        else
            echo -e "${YELLOW}⚠️ El modelo 'mistral' no está disponible${NC}"
            echo -e "${CYAN}❓ ¿Deseas descargar el modelo 'mistral'? (s/n)${NC}"
            read -p "> " download_mistral
            
            if [ "$download_mistral" = "s" ] || [ "$download_mistral" = "S" ]; then
                echo -e "${YELLOW}📦 Descargando modelo 'mistral'...${NC}"
                if ollama pull mistral; then
                    echo -e "${GREEN}✅ Modelo 'mistral' descargado correctamente${NC}"
                else
                    echo -e "${RED}❌ Error al descargar el modelo 'mistral'${NC}"
                    echo -e "${YELLOW}📝 El chatbot no funcionará correctamente sin este modelo${NC}"
                    return 1
                fi
            else
                echo -e "${YELLOW}⚠️ Continuando sin descargar el modelo 'mistral'. El chatbot no funcionará correctamente.${NC}"
                return 1
            fi
        fi
    fi
    
    # Verificar conexión con el endpoint raíz de Ollama
    echo -e "${YELLOW}🔍 Verificando conexión con Ollama...${NC}"
    if curl -s http://localhost:11434/ | grep -q "Ollama is running"; then
        echo -e "${GREEN}✅ Servicio Ollama responde correctamente${NC}"
    else
        echo -e "${RED}❌ No se puede conectar con el servicio Ollama${NC}"
        echo -e "${YELLOW}📝 Verifica que el servicio esté en ejecución con: ps aux | grep ollama${NC}"
        echo -e "${YELLOW}📝 Intenta reiniciar el servicio o ejecutarlo manualmente con: ollama serve${NC}"
        return 1
    fi
    
    return 0
}

# Función para verificar si ROS2 está instalado y configurado
check_ros() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}❌ ROS2 no está instalado o no está en el PATH${NC}"
        echo -e "${YELLOW}📝 Por favor, asegúrate de que ROS2 está instalado y que has ejecutado:${NC}"
        echo -e "${YELLOW}   source /opt/ros/galactic/setup.bash${NC}"
        exit 1
    fi
}

# Función para verificar e instalar un paquete ROS2
check_ros_package() {
    local package=$1
    local apt_package=$2
    local is_optional=${3:-false}
    
    echo -e "${YELLOW}🔍 Verificando paquete ROS2: ${CYAN}$package${NC}"
    
    # Verificar si el paquete está instalado
    if ! ros2 pkg list | grep -q "$package"; then
        if [ "$is_optional" = true ]; then
            echo -e "${YELLOW}⚠️ El paquete opcional '$package' no está instalado${NC}"
            echo -e "${YELLOW}📝 Este paquete es opcional, el sistema funcionará sin él${NC}"
            return 1
        else
            echo -e "${YELLOW}⚠️ El paquete '$package' no está instalado${NC}"
            echo -e "${CYAN}❓ ¿Deseas instalar el paquete '$apt_package'? (s/n)${NC}"
            read -p "> " install_pkg
            
            if [ "$install_pkg" = "s" ] || [ "$install_pkg" = "S" ]; then
                echo -e "${YELLOW}📦 Intentando instalar $apt_package...${NC}"
                if sudo apt update && sudo apt install -y "$apt_package"; then
                    # Verificar si se instaló correctamente
                    if ros2 pkg list | grep -q "$package"; then
                        echo -e "${GREEN}✅ Paquete '$package' instalado correctamente${NC}"
                        return 0
                    else
                        echo -e "${RED}❌ Error al instalar el paquete '$package'${NC}"
                        if [ "$package" = "web_video_server" ]; then
                            echo -e "${YELLOW}📝 Nota: El paquete web_video_server podría tener otro nombre en tu distribución${NC}"
                            echo -e "${YELLOW}📝 Intenta buscar el paquete correcto con: apt search web_video_server${NC}"
                            echo -e "${YELLOW}📝 La aplicación puede funcionar sin este paquete${NC}"
                        fi
                        return 1
                    fi
                else
                    echo -e "${RED}❌ Error al instalar el paquete '$apt_package'${NC}"
                    if [ "$package" = "web_video_server" ]; then
                        echo -e "${YELLOW}📝 Nota: El paquete podría no estar disponible para tu versión de ROS2${NC}"
                        echo -e "${YELLOW}📝 Prueba con: sudo apt install ros-\$(rosversion -d)-web-video-server${NC}"
                        echo -e "${YELLOW}📝 O busca el paquete correcto con: apt search web_video_server${NC}"
                        echo -e "${YELLOW}📝 La aplicación puede funcionar sin este paquete${NC}"
                    fi
                    return 1
                fi
            else
                if [ "$package" = "web_video_server" ]; then
                    echo -e "${YELLOW}📝 La aplicación puede funcionar sin este paquete${NC}"
                else
                    echo -e "${RED}❌ Se requiere el paquete '$package' para esta funcionalidad${NC}"
                fi
                return 1
            fi
        fi
    else
        echo -e "${GREEN}✅ Paquete '$package' ya está instalado${NC}"
        return 0
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

# Verificar instalación de Node.js y npm
check_and_install_nodejs
NODEJS_OK=$?

# Verificar instalación de Ollama
check_and_install_ollama
OLLAMA_OK=$?

# Verificar dependencia js-yaml
check_and_install_jsyaml

# Verificar paquetes ROS2 necesarios
echo -e "${YELLOW}🔍 Verificando paquetes ROS2 necesarios...${NC}"
check_ros_package "rosbridge_server" "ros-galactic-rosbridge-server"
ROSBRIDGE_OK=$?
check_ros_package "web_video_server" "ros-galactic-web-video-server" true
WEB_VIDEO_OK=$?
echo

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

# Terminal 6: ROS Bridge Server
TERMINAL6_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${MAGENTA}╔════════════════════════════════════════╗${NC}\" && \
    echo -e \"${MAGENTA}║  TERMINAL 6: ROS BRIDGE SERVER         ║${NC}\" && \
    echo -e \"${MAGENTA}╚════════════════════════════════════════╝${NC}\" && \
    echo -e \"${CYAN}🌉 Lanzando ROS Bridge Server...${NC}\" && \
    if ros2 pkg list | grep -q \"rosbridge_server\"; then \
        ros2 launch rosbridge_server rosbridge_websocket_launch.xml; \
    else \
        echo -e \"${RED}❌ El paquete 'rosbridge_server' no está instalado.${NC}\" && \
        echo -e \"${YELLOW}📝 Instálalo con: sudo apt install ros-galactic-rosbridge-server${NC}\" && \
        echo -e \"${YELLOW}📝 Este paquete es necesario para la comunicación entre ROS2 y el navegador.${NC}\" && \
        echo -e \"${YELLOW}📝 Sin este paquete, la interfaz web no podrá comunicarse con ROS2.${NC}\"; \
    fi && \
    read -p \"Presiona Enter para cerrar esta terminal...\""

# Terminal 7: Web Video Server
TERMINAL7_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${BLUE}╔════════════════════════════════════════╗${NC}\" && \
    echo -e \"${BLUE}║  TERMINAL 7: WEB VIDEO SERVER          ║${NC}\" && \
    echo -e \"${BLUE}╚════════════════════════════════════════╝${NC}\" && \
    echo -e \"${CYAN}🎥 Lanzando Web Video Server...${NC}\" && \
    if ros2 pkg list | grep -q \"web_video_server\"; then \
        ros2 run web_video_server web_video_server; \
    else \
        ROS_DISTRO=\$(rosversion -d 2>/dev/null || echo \"galactic\") && \
        echo -e \"${YELLOW}⚠️ El paquete 'web_video_server' no está instalado.${NC}\" && \
        echo -e \"${YELLOW}📝 Puedes intentar instalarlo con uno de estos comandos:${NC}\" && \
        echo -e \"${YELLOW}   sudo apt install ros-\$ROS_DISTRO-web-video-server${NC}\" && \
        echo -e \"${YELLOW}   sudo apt install ros-\$ROS_DISTRO-web-video-server-dbgsym${NC}\" && \
        echo -e \"${CYAN}ℹ️ NOTA: No te preocupes, la cámara puede funcionar a través de otros métodos:${NC}\" && \
        echo -e \"${CYAN}   - A través de rosbridge_server (websockets)${NC}\" && \
        echo -e \"${CYAN}   - Usando la implementación propia del frontend${NC}\" && \
        echo -e \"${CYAN}   - O mediante otras bibliotecas JavaScript como roslibjs${NC}\" && \
        echo -e \"${GREEN}✅ El sistema seguirá funcionando sin este componente${NC}\"; \
    fi && \
    read -p \"Presiona Enter para cerrar esta terminal...\""

# Terminal 8: Install ROS Libraries for Web
TERMINAL8_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${YELLOW}╔════════════════════════════════════════╗${NC}\" && \
    echo -e \"${YELLOW}║  TERMINAL 8: FRONTEND WEB              ║${NC}\" && \
    echo -e \"${YELLOW}╚════════════════════════════════════════╝${NC}\" && \
    if [ -d \"$WORKSPACE_PATH/src/aidguide_04_web\" ]; then \
        echo -e \"${CYAN}📦 Navegando al directorio del frontend...${NC}\" && \
        cd \"$WORKSPACE_PATH/src/aidguide_04_web\" && \
        echo -e \"${CYAN}📦 Instalando dependencias necesarias...${NC}\" && \
        npm install --save js-yaml --legacy-peer-deps && \
        echo -e \"${GREEN}✅ js-yaml instalado correctamente en el frontend${NC}\" && \
        echo -e \"${CYAN}🚀 Iniciando servidor de desarrollo...${NC}\" && \
        npm run dev; \
    else \
        echo -e \"${YELLOW}⚠️ Directorio del frontend no encontrado en $WORKSPACE_PATH/src/aidguide_04_web${NC}\" && \
        echo -e \"${CYAN}📦 Instalando dependencias globalmente...${NC}\" && \
        npm install -g roslib js-yaml; \
    fi && \
    read -p \"Presiona Enter para cerrar esta terminal...\""

# Terminal 9: Monitor de Batería y Sensores
TERMINAL9_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${GREEN}╔════════════════════════════════════════════╗${NC}\" && \
    echo -e \"${GREEN}║  TERMINAL 9: MONITOR DE SENSORES       ║${NC}\" && \
    echo -e \"${GREEN}╚════════════════════════════════════════════╝${NC}\" && \
    echo -e \"${YELLOW}🕒 Esperando 15 segundos para que todos los servicios estén activos...${NC}\" && \
    sleep 15 && \
    echo -e \"${YELLOW}🔍 Verificando tópicos del sistema...${NC}\" && \
    TOPICS_TO_CHECK=(\"/battery_status\" \"/hardware_health\" \"/temperature_sensor\" \"/log_messages\") && \
    AVAILABLE_TOPICS=() && \
    
    # Función para verificar y mostrar información de un tópico
    check_topic() {
        local topic=\$1
        if ros2 topic list 2>/dev/null | grep -q \"\$topic\"; then
            echo -e \"${GREEN}✅ Tópico encontrado: \$topic${NC}\"
            echo -e \"${CYAN}ℹ️ Información del tópico:${NC}\"
            ros2 topic info \$topic
            AVAILABLE_TOPICS+=(\"\$topic\")
            return 0
        else
            echo -e \"${YELLOW}⚠️ Tópico no encontrado: \$topic${NC}\"
            return 1
        fi
    } && \
    
    # Verificar cada tópico
    for topic in \"\${TOPICS_TO_CHECK[@]}\"; do
        check_topic \$topic
    done && \
    
    if [ \${#AVAILABLE_TOPICS[@]} -eq 0 ]; then
        echo -e \"${YELLOW}⚠️ No se encontraron los tópicos específicos. Esperando 10 segundos más...${NC}\" && \
        sleep 10 && \
        for topic in \"\${TOPICS_TO_CHECK[@]}\"; do
            check_topic \$topic
        done
    fi && \
    
    if [ \${#AVAILABLE_TOPICS[@]} -eq 0 ]; then
        echo -e \"${RED}❌ No se encontraron los tópicos específicos después de esperar${NC}\" && \
        echo -e \"${YELLOW}📝 Mostrando todos los tópicos disponibles:${NC}\" && \
        ros2 topic list
    else
        echo -e \"${GREEN}✅ Tópicos disponibles: \${AVAILABLE_TOPICS[@]}${NC}\" && \
        
        # Monitorizar cada tópico encontrado brevemente
        for topic in \"\${AVAILABLE_TOPICS[@]}\"; do
            echo -e \"${CYAN}🔄 Monitorizando \$topic (5 segundos o 1 mensaje)...${NC}\"
            timeout 5s ros2 topic echo \$topic --once 2>/dev/null || echo -e \"${YELLOW}⚠️ Sin mensajes en \$topic durante el tiempo de espera${NC}\"
            echo
        done && \
        
        # Menú interactivo para monitoreo continuo
        echo -e \"${CYAN}╔════════════════════════════════════════╗${NC}\" && \
        echo -e \"${CYAN}║  MONITOR CONTINUO DE TÓPICOS           ║${NC}\" && \
        echo -e \"${CYAN}╚════════════════════════════════════════╝${NC}\" && \
        echo -e \"${YELLOW}Selecciona un tópico para monitorizar continuamente:${NC}\" && \
        
        # Mostrar opciones de tópicos disponibles
        for i in \"\${!AVAILABLE_TOPICS[@]}\"; do
            echo -e \"${GREEN}   \$((i+1))) \${AVAILABLE_TOPICS[i]}${NC}\"
        done && \
        echo -e \"${GREEN}   q) Salir${NC}\" && \
        
        read -p \"> \" selection && \
        
        if [[ \$selection =~ ^[0-9]+\$ ]] && [ \$selection -ge 1 ] && [ \$selection -le \${#AVAILABLE_TOPICS[@]} ]; then
            selected_topic=\"\${AVAILABLE_TOPICS[\$((selection-1))]}\"
            echo -e \"${CYAN}🔄 Monitorizando \$selected_topic continuamente. Presiona Ctrl+C para detener.${NC}\"
            ros2 topic echo \$selected_topic
        else
            echo -e \"${YELLOW}📝 Saliendo del monitor de tópicos${NC}\"
        fi
    fi && \
    echo -e \"${CYAN}🔄 Monitoreo de sensores finalizado${NC}\" && \
    echo -e \"${CYAN}📝 Presiona Enter para cerrar esta terminal${NC}\" && \
    read -p \"> \" dummy"

# Terminal 10: Puente Web-Waypoint
TERMINAL10_COMMANDS="cd \"$WORKSPACE_PATH\" && \
    echo -e \"${RED}╔════════════════════════════════════════════╗${NC}\" && \
    echo -e \"${RED}║  TERMINAL 10: PUENTE WEB-WAYPOINT      ║${NC}\" && \
    echo -e \"${RED}╚════════════════════════════════════════════╝${NC}\" && \
    echo -e \"${YELLOW}🔨 Compilando nodo puente...${NC}\" && \
    colcon build --packages-select aidguide_04_my_nav2_system && \
    echo -e \"${YELLOW}🔄 Actualizando entorno...${NC}\" && \
    source install/setup.bash && \
    echo -e \"${CYAN}🌉 Iniciando nodo puente web-waypoint...${NC}\" && \
    ros2 run aidguide_04_my_nav2_system web_waypoint_bridge && \
    read -p \"Presiona Enter para cerrar esta terminal...\""

# Terminal 11: Servicio Ollama
TERMINAL11_COMMANDS="(echo -e \"${MAGENTA}║  TERMINAL 11: SERVICIO OLLAMA            ║${NC}\" && \
echo && \
if pgrep -f ollama > /dev/null; then \
  echo -e \"${CYAN}🤖 Ollama ya está en ejecución...${NC}\" && \
  echo -e \"${YELLOW}📝 Si experimentas problemas, reinicia el servicio con: ollama serve${NC}\"; \
else \
  echo -e \"${CYAN}🤖 Iniciando servicio Ollama...${NC}\" && \
  ollama serve; \
fi)"

# Terminal 12: Chatbot con Ollama
TERMINAL12_COMMANDS="(echo -e \"${MAGENTA}║  TERMINAL 12: CHATBOT CON OLLAMA         ║${NC}\" && \
echo && \
if [ $NODEJS_OK -eq 0 ] && [ $OLLAMA_OK -eq 0 ]; then \
  echo -e \"${CYAN}🤖 Iniciando el chatbot con Ollama...${NC}\" && \
  if [ ! -d \"$WORKSPACE_PATH/src/aidguide_04_web\" ]; then \
    echo -e \"${RED}❌ No se encontró el directorio del frontend web en $WORKSPACE_PATH/src/aidguide_04_web${NC}\"; \
  else \
    cd \"$WORKSPACE_PATH/src/aidguide_04_web\" && \
    echo -e \"${CYAN}📦 Verificando dependencias...${NC}\" && \
    if [ ! -d \"node_modules\" ] || [ ! -f \"node_modules/.package-lock.json\" ]; then \
      echo -e \"${YELLOW}📦 Instalando dependencias...${NC}\" && \
      npm install; \
    fi && \
    echo -e \"${CYAN}🚀 Iniciando el servidor web con chatbot...${NC}\" && \
    npm run dev; \
  fi \
else \
  if [ $NODEJS_OK -ne 0 ]; then \
    echo -e \"${RED}❌ Node.js o npm no están disponibles. No se puede iniciar el chatbot.${NC}\"; \
  fi && \
  if [ $OLLAMA_OK -ne 0 ]; then \
    echo -e \"${RED}❌ Ollama no está disponible. No se puede iniciar el chatbot.${NC}\"; \
  fi && \
  echo -e \"${YELLOW}📝 Para iniciar el chatbot manualmente:${NC}\" && \
  echo -e \"${YELLOW}   1. Instala Node.js y npm${NC}\" && \
  echo -e \"${YELLOW}   2. Instala Ollama y descarga el modelo 'mistral'${NC}\" && \
  echo -e \"${YELLOW}   3. Ejecuta 'cd $WORKSPACE_PATH/src/aidguide_04_web && npm run dev'${NC}\"; \
fi)"

# Mostrar instrucciones de inicio
echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${YELLOW}🚀 INICIANDO SISTEMA DE NAVEGACIÓN                     ${CYAN}  ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${GREEN}Se abrirán 13 terminales con los diferentes componentes ${CYAN}  ║${NC}"
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

# Iniciamos los servicios web
start_terminal "Terminal 5: ROS Bridge Server" "$TERMINAL6_COMMANDS" "${MAGENTA}"
sleep 3

start_terminal "Terminal 6: Web Video Server" "$TERMINAL7_COMMANDS" "${BLUE}"
sleep 3

start_terminal "Terminal 7: Frontend Web" "$TERMINAL8_COMMANDS" "${YELLOW}"
sleep 3

# Gazebo es el penúltimo en iniciarse
start_terminal "Terminal 8: Gazebo" "$TERMINAL1_COMMANDS" "${BLUE}"
echo -e "${YELLOW}🕒 Esperando 10 segundos para que Gazebo se inicialice completamente...${NC}"
sleep 10

# Monitor de batería es el último en iniciarse
start_terminal "Terminal 9: Monitor de Sensores" "$TERMINAL9_COMMANDS" "${GREEN}"

# Terminal 10: Puente Web-Waypoint
start_terminal "Terminal 10: Puente Web-Waypoint" "$TERMINAL10_COMMANDS" "${RED}"

# Terminal 11: Servicio Ollama
start_terminal "Terminal 11: Servicio Ollama" "$TERMINAL11_COMMANDS" "${MAGENTA}"

# Terminal 12: Chatbot con Ollama
if [ $NODEJS_OK -eq 0 ] && [ $OLLAMA_OK -eq 0 ]; then
  start_terminal "Terminal 12: Chatbot con Ollama" "$TERMINAL12_COMMANDS" "${MAGENTA}"
  echo -e "${GREEN}✅ Chatbot iniciado en http://localhost:3000${NC}"
else
  echo -e "${YELLOW}⚠️ No se pudo iniciar el chatbot debido a dependencias faltantes${NC}"
fi

echo ""
echo -e "${GREEN}✅ Todos los componentes han sido iniciados${NC}"
echo -e "${YELLOW}📝 Para interactuar con la navegación, utiliza RViz y las herramientas proporcionadas${NC}"
if [ $ROSBRIDGE_OK -eq 0 ]; then
    echo -e "${CYAN}🌐 Puedes acceder a la interfaz web a través del navegador cuando el frontend esté iniciado${NC}"
else
    echo -e "${YELLOW}⚠️ La comunicación con el navegador (frontend) no estará disponible hasta que instales rosbridge_server${NC}"
fi
echo ""
echo -e "${CYAN}🌐 Puedes acceder al chatbot inteligente desde la interfaz web cuando esté iniciado${NC}"
echo -e "${YELLOW}💡 El chatbot utiliza el modelo 'mistral' de Ollama para responder consultas sobre AidGuide${NC}"
echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${GREEN}Sistema completo en funcionamiento                     ${CYAN}  ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}" 

# Nota: Los archivos originales se restaurarán automáticamente cuando se cierre el script
# gracias al comando trap configurado anteriormente 