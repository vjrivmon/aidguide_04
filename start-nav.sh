#!/bin/bash

# Script de automatización para iniciar el paquete de navegación aidguide_04_nav
# Autor: DevOps Team
# Fecha: 2024
# Descripción: Este script automatiza la compilación y ejecución del paquete de navegación

# Colores para los mensajes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
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

# Función para crear el workspace si no existe
create_workspace() {
    local workspace_path="$HOME/aidguide_04/aidguide_04_ws"
    if [ ! -d "$workspace_path" ]; then
        echo -e "${YELLOW}📂 Creando workspace en $workspace_path...${NC}"
        mkdir -p "$workspace_path/src"
        cd "$workspace_path" || exit 1
        colcon build
    fi
    echo -e "${GREEN}✅ Workspace verificado${NC}"
}

echo -e "${CYAN}🚀 Iniciando el proceso de configuración de navegación...${NC}"

# Verificar ROS2
check_ros

# Determinar la ruta del workspace
WORKSPACE_PATH="$HOME/aidguide_04/aidguide_04_ws"
CURRENT_WORKSPACE_PATH="$HOME/Desktop/aidguide_04/aidguide_04_ws"

# Intentar diferentes ubicaciones del workspace
if [ -d "$WORKSPACE_PATH" ]; then
    echo -e "${YELLOW}📂 Navegando al workspace principal...${NC}"
    cd "$WORKSPACE_PATH" || exit 1
elif [ -d "$CURRENT_WORKSPACE_PATH" ]; then
    echo -e "${YELLOW}📂 Navegando al workspace en Desktop...${NC}"
    cd "$CURRENT_WORKSPACE_PATH" || exit 1
else
    echo -e "${YELLOW}📂 Creando nuevo workspace...${NC}"
    create_workspace
    WORKSPACE_PATH="$HOME/aidguide_04/aidguide_04_ws"
    cd "$WORKSPACE_PATH" || exit 1
fi

# Verificar y crear el paquete de navegación si no existe
NAV_PATH="src/aidguide_04_nav"
if [ ! -d "$NAV_PATH" ]; then
    echo -e "${YELLOW}📦 Creando paquete de navegación...${NC}"
    cd src || exit 1
    ros2 pkg create --build-type ament_python aidguide_04_nav
    cd .. || exit 1
fi

# Compilar el paquete
echo -e "${YELLOW}🔨 Compilando el paquete...${NC}"
colcon build --packages-select aidguide_04_nav || {
    echo -e "${RED}❌ Error al compilar el paquete${NC}"
    exit 1
}

# Verificar y establecer permisos de los archivos Python
echo -e "${YELLOW}🔑 Verificando permisos de archivos...${NC}"
NAV_SCRIPT="$NAV_PATH/aidguide_04_nav/aidguide_04_nav.py"
LAUNCH_FILE="$NAV_PATH/launch/aidguide_04_nav_launch.launch.py"

# Crear archivos si no existen
if [ ! -f "$NAV_SCRIPT" ]; then
    echo -e "${YELLOW}📝 Creando archivo de navegación...${NC}"
    mkdir -p "$(dirname "$NAV_SCRIPT")"
    cat > "$NAV_SCRIPT" << 'EOF'
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AidguideNavigation(Node):
    def __init__(self):
        super().__init__('aidguide_navigation')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.5
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = AidguideNavigation()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
    chmod +x "$NAV_SCRIPT"
    echo -e "${GREEN}✅ Archivo de navegación creado${NC}"
fi

if [ ! -f "$LAUNCH_FILE" ]; then
    echo -e "${YELLOW}📝 Creando archivo launch...${NC}"
    mkdir -p "$(dirname "$LAUNCH_FILE")"
    cat > "$LAUNCH_FILE" << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aidguide_04_nav',
            executable='aidguide_navigation',
            output='screen'),
    ])
EOF
    chmod +x "$LAUNCH_FILE"
    echo -e "${GREEN}✅ Archivo launch creado${NC}"
fi

# Verificar permisos
chmod +x "$NAV_SCRIPT" 2>/dev/null || true
chmod +x "$LAUNCH_FILE" 2>/dev/null || true

# Source el workspace
echo -e "${YELLOW}🔄 Actualizando entorno...${NC}"
source /opt/ros/galactic/setup.bash
source install/setup.bash

# Iniciar la simulación en una nueva terminal
echo -e "${CYAN}🌐 Iniciando simulación Gazebo...${NC}"
gnome-terminal -- bash -c "source /opt/ros/galactic/setup.bash && source install/setup.bash && ros2 launch turtlebot3_gazebo empty_world.launch.py" || \
xterm -e "source /opt/ros/galactic/setup.bash && source install/setup.bash && ros2 launch turtlebot3_gazebo empty_world.launch.py" || \
konsole -e "source /opt/ros/galactic/setup.bash && source install/setup.bash && ros2 launch turtlebot3_gazebo empty_world.launch.py" || {
    echo -e "${RED}❌ No se pudo iniciar Gazebo${NC}"
    exit 1
}

# Esperamos unos segundos para que Gazebo se inicie
echo -e "${YELLOW}⏳ Esperando a que Gazebo se inicie...${NC}"
sleep 10

# Iniciar la navegación
echo -e "${CYAN}🗺️ Iniciando sistema de navegación...${NC}"
ros2 launch aidguide_04_nav aidguide_04_nav_launch.launch.py 