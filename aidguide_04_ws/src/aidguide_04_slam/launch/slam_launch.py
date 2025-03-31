#!/usr/bin/env python3
"""
Archivo de lanzamiento para iniciar el SLAM en AidGuide 04.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Genera la descripción de lanzamiento para el SLAM.
    """
    # Argumentos de lanzamiento
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Directorio de recursos del paquete
    pkg_share = get_package_share_directory('aidguide_04_slam')
    
    # Configuración para slam_toolbox
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    
    # Configuración del entorno
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # Nodos a lanzar
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )
    
    aidguide_slam_node = Node(
        package='aidguide_04_slam',
        executable='slam_node',
        name='aidguide_slam_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Crear y retornar el lanzamiento
    return LaunchDescription([
        # Argumentos
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar tiempo de simulación si es true'),
            
        # Configuración del entorno
        stdout_linebuf_envvar,
        
        # Nodos
        slam_toolbox_node,
        aidguide_slam_node,
    ]) 