#!/usr/bin/env python3
"""
Archivo de lanzamiento para visualizar un mapa existente en AidGuide 04.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Genera la descripción de lanzamiento para visualizar un mapa.
    """
    # Argumentos de lanzamiento
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file', default=os.path.join(
        get_package_share_directory('aidguide_04_slam'),
        'maps',
        'tb3_house_map.yaml'))
    
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
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ],
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
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
                'tb3_house_map.yaml'),
            description='Ruta completa al archivo de mapa para cargar'),
            
        # Nodos
        map_server_node,
        lifecycle_manager_node,
        rviz_node,
    ]) 