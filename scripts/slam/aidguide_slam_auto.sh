#!/bin/bash
#
# Script automatizado para AidGuide 04 SLAM
# Este script facilita la ejecución de tareas de SLAM y localización
#

# Colores para mensajes
ROJO='\033[0;31m'
VERDE='\033[0;32m'
AMARILLO='\033[0;33m'
AZUL='\033[0;34m'
NC='\033[0m' # Sin Color

# Configuración de directorios
WORKSPACE_DIR="$HOME/aidguide_04"
PACKAGE_DIR="$WORKSPACE_DIR/aidguide_04_ws/src/aidguide_04_slam"
MAPS_DIR="$PACKAGE_DIR/maps"

# Banner de inicio
function mostrar_banner {
    clear
    echo -e "${AZUL}=============================================${NC}"
    echo -e "${VERDE}      AidGuide 04 - Asistente de SLAM       ${NC}"
    echo -e "${AZUL}=============================================${NC}"
    echo ""
}

# Función para verificar dependencias
function verificar_dependencias {
    echo -e "${AMARILLO}Verificando dependencias...${NC}"
    
    # Verificar ROS2 Galactic
    if ! command -v ros2 &> /dev/null; then
        echo -e "${ROJO}Error: ROS2 no está instalado o no está en el PATH${NC}"
        exit 1
    fi
    
    # Verificar paquetes necesarios
    for pkg in slam_toolbox nav2_map_server nav2_amcl nav2_lifecycle_manager; do
        if ! ros2 pkg list | grep -q $pkg; then
            echo -e "${ROJO}Error: El paquete $pkg no está instalado.${NC}"
            echo -e "Instálelo con: sudo apt install ros-galactic-$pkg"
            exit 1
        fi
    done
    
    # Verificar que TURTLEBOT3_MODEL esté configurado
    if [ -z "$TURTLEBOT3_MODEL" ]; then
        export TURTLEBOT3_MODEL=waffle
        echo -e "${AMARILLO}Variable TURTLEBOT3_MODEL no configurada. Establecida a 'waffle'.${NC}"
    else
        echo -e "${VERDE}TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL${NC}"
    fi
    
    echo -e "${VERDE}Todas las dependencias están satisfechas.${NC}"
}

# Función para compilar el paquete
function compilar_paquete {
    echo -e "${AMARILLO}Compilando el paquete aidguide_04_slam...${NC}"
    cd $WORKSPACE_DIR
    if colcon build --packages-select aidguide_04_slam; then
        echo -e "${VERDE}Compilación exitosa.${NC}"
    else
        echo -e "${ROJO}Error en la compilación.${NC}"
        exit 1
    fi
    
    # Cargar el workspace
    source $WORKSPACE_DIR/install/setup.bash
}

# Función para listar mapas disponibles
function listar_mapas {
    echo -e "${AZUL}Mapas disponibles:${NC}"
    local count=1
    
    # Verificar que el directorio de mapas existe
    if [ ! -d "$MAPS_DIR" ]; then
        echo -e "${ROJO}Error: Directorio de mapas no encontrado.${NC}"
        return 1
    fi
    
    # Listar los mapas YAML
    for map_file in $MAPS_DIR/*.yaml; do
        if [ -f "$map_file" ]; then
            map_name=$(basename "$map_file" .yaml)
            echo -e "  ${VERDE}$count.${NC} $map_name"
            count=$((count+1))
        fi
    done
    
    if [ $count -eq 1 ]; then
        echo -e "${AMARILLO}No se encontraron mapas.${NC}"
        return 1
    fi
    
    return 0
}

# Función para seleccionar un mapa
function seleccionar_mapa {
    listar_mapas
    
    if [ $? -ne 0 ]; then
        echo -e "${AMARILLO}Usando configuración por defecto.${NC}"
        return ""
    fi
    
    echo ""
    echo -e "${AZUL}Seleccione un mapa por número o presione Enter para usar el predeterminado:${NC}"
    read seleccion
    
    if [ -z "$seleccion" ]; then
        echo -e "${AMARILLO}Usando mapa predeterminado.${NC}"
        return ""
    fi
    
    local count=1
    for map_file in $MAPS_DIR/*.yaml; do
        if [ -f "$map_file" ]; then
            if [ $count -eq $seleccion ]; then
                map_name=$(basename "$map_file" .yaml)
                echo -e "${VERDE}Mapa seleccionado: $map_name${NC}"
                echo "$map_name"
                return
            fi
            count=$((count+1))
        fi
    done
    
    echo -e "${ROJO}Selección inválida. Usando mapa predeterminado.${NC}"
    return ""
}

# Función para visualizar mapa
function visualizar_mapa {
    mapa=$(seleccionar_mapa)
    
    echo -e "${AMARILLO}Iniciando visualización del mapa...${NC}"
    
    if [ -n "$mapa" ]; then
        map_path="$(ros2 pkg prefix aidguide_04_slam)/share/aidguide_04_slam/maps/$mapa.yaml"
        ros2 launch aidguide_04_slam test_map_with_robot.launch.py map_file:=$map_path
    else
        ros2 launch aidguide_04_slam test_map_with_robot.launch.py
    fi
}

# Función para ejecutar SLAM
function ejecutar_slam {
    echo -e "${AMARILLO}Iniciando SLAM...${NC}"
    ros2 launch aidguide_04_slam test_slam.launch.py
}

# Función para guardar mapa
function guardar_mapa {
    echo -e "${AZUL}Introduzca un nombre para el mapa (sin extensión):${NC}"
    read nombre_mapa
    
    if [ -z "$nombre_mapa" ]; then
        echo -e "${ROJO}Nombre de mapa inválido.${NC}"
        return
    fi
    
    echo -e "${AMARILLO}Guardando mapa como $nombre_mapa...${NC}"
    
    # Crear directorio temporal para guardar el mapa
    mkdir -p /tmp/aidguide_maps
    
    # Guardar el mapa
    ros2 run nav2_map_server map_saver_cli -f /tmp/aidguide_maps/$nombre_mapa
    
    # Verificar que se creó correctamente
    if [ -f "/tmp/aidguide_maps/$nombre_mapa.pgm" ] && [ -f "/tmp/aidguide_maps/$nombre_mapa.yaml" ]; then
        # Copiar al directorio de mapas del paquete
        mkdir -p $MAPS_DIR
        cp /tmp/aidguide_maps/$nombre_mapa.* $MAPS_DIR/
        
        echo -e "${VERDE}Mapa guardado exitosamente en $MAPS_DIR${NC}"
        echo -e "${AMARILLO}Recompilando el paquete para incluir el nuevo mapa...${NC}"
        
        # Recompilar el paquete
        compilar_paquete
    else
        echo -e "${ROJO}Error al guardar el mapa.${NC}"
    fi
}

# Función para ejecutar localización
function ejecutar_localizacion {
    mapa=$(seleccionar_mapa)
    
    echo -e "${AMARILLO}Iniciando localización...${NC}"
    
    if [ -n "$mapa" ]; then
        map_path="$(ros2 pkg prefix aidguide_04_slam)/share/aidguide_04_slam/maps/$mapa.yaml"
        ros2 launch aidguide_04_slam test_map_with_robot.launch.py map_file:=$map_path
    else
        ros2 launch aidguide_04_slam test_map_with_robot.launch.py
    fi
}

# Verificar argumentos
function mostrar_ayuda {
    echo -e "${AZUL}Uso: $0 [opción]${NC}"
    echo -e "${VERDE}Opciones disponibles:${NC}"
    echo -e "  ${AMARILLO}slam${NC}         Iniciar SLAM para crear un nuevo mapa"
    echo -e "  ${AMARILLO}save${NC}         Guardar el mapa actual"
    echo -e "  ${AMARILLO}localize${NC}     Ejecutar localización en un mapa existente"
    echo -e "  ${AMARILLO}view${NC}         Visualizar un mapa existente"
    echo -e "  ${AMARILLO}help${NC}         Mostrar esta ayuda"
    echo ""
    echo -e "${AZUL}Ejemplos:${NC}"
    echo -e "  $0 slam"
    echo -e "  $0 save"
    echo -e "  $0 localize"
}

# Menú principal
function menu_principal {
    mostrar_banner
    
    echo -e "${VERDE}Opciones disponibles:${NC}"
    echo -e "  ${AMARILLO}1.${NC} Iniciar SLAM (crear nuevo mapa)"
    echo -e "  ${AMARILLO}2.${NC} Guardar mapa actual"
    echo -e "  ${AMARILLO}3.${NC} Ejecutar localización en mapa existente"
    echo -e "  ${AMARILLO}4.${NC} Visualizar mapa existente"
    echo -e "  ${AMARILLO}0.${NC} Salir"
    echo ""
    echo -e "${AZUL}Seleccione una opción:${NC}"
    read opcion
    
    case $opcion in
        1)
            ejecutar_slam
            ;;
        2)
            guardar_mapa
            ;;
        3)
            ejecutar_localizacion
            ;;
        4)
            visualizar_mapa
            ;;
        0)
            echo -e "${VERDE}¡Hasta pronto!${NC}"
            exit 0
            ;;
        *)
            echo -e "${ROJO}Opción inválida${NC}"
            sleep 2
            menu_principal
            ;;
    esac
}

# Flujo principal
verificar_dependencias
compilar_paquete

# Si se proporcionan argumentos, procesarlos
if [ $# -gt 0 ]; then
    case $1 in
        slam)
            ejecutar_slam
            ;;
        save)
            guardar_mapa
            ;;
        localize)
            ejecutar_localizacion
            ;;
        view)
            visualizar_mapa
            ;;
        help|--help|-h)
            mostrar_ayuda
            ;;
        *)
            echo -e "${ROJO}Opción inválida: $1${NC}"
            mostrar_ayuda
            exit 1
            ;;
    esac
else
    # Sin argumentos, mostrar el menú interactivo
    menu_principal
fi 