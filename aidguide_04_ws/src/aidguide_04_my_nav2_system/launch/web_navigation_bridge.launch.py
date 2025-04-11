#!/usr/bin/env python3

"""
Archivo de lanzamiento para iniciar el nodo puente entre la interfaz web y el seguidor de waypoints
Autor: Claude
Fecha: 2024
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    LogInfo
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """Crear la descripción de lanzamiento para este paquete"""
    
    pkg_dir = get_package_share_directory('aidguide_04_my_nav2_system')
    
    # Lanzar el puente web-waypoint
    web_waypoint_bridge_node = Node(
        package='aidguide_04_my_nav2_system',
        executable='web_waypoint_bridge',
        name='web_waypoint_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[]
    )

    # Asegurarnos de que rosbridge server esté en ejecución
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'
        )])
    )
    
    # Mensaje informativo
    startup_info = LogInfo(
        msg=["Iniciando sistema de puente entre web y navegación por waypoints..."]
    )

    return LaunchDescription([
        startup_info,
        rosbridge_launch,
        web_waypoint_bridge_node,
    ]) 