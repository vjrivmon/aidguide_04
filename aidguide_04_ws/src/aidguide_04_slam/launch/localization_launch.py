#!/usr/bin/env python3
"""
Archivo de lanzamiento para iniciar la localización en AidGuide 04.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Genera la descripción de lanzamiento para la localización.
    """
    # Argumentos de lanzamiento
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file', default=os.path.join(
        get_package_share_directory('aidguide_04_slam'),
        'maps',
        'map.yaml'))
    
    # Directorio de recursos del paquete
    pkg_share = get_package_share_directory('aidguide_04_slam')
    
    # Configuración para AMCL
    amcl_params_file = os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    
    # Configuración del entorno
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    
    # Nodos a lanzar
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ],
    )
    
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ],
    )
    
    aidguide_localization_node = Node(
        package='aidguide_04_slam',
        executable='localization_node',
        name='aidguide_localization_node',
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
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(
                get_package_share_directory('aidguide_04_slam'),
                'maps',
                'map.yaml'),
            description='Ruta completa al archivo de mapa para cargar'),
            
        # Configuración del entorno
        stdout_linebuf_envvar,
        
        # Nodos
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        aidguide_localization_node,
    ]) 