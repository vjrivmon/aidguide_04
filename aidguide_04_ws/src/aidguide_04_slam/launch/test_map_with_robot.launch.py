#!/usr/bin/env python3
"""
Archivo de lanzamiento para probar el mapa con un robot simulado.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Genera la descripción de lanzamiento para probar el mapa con un robot.
    """
    # Parámetros
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map_file', default=os.path.join(
        get_package_share_directory('aidguide_04_slam'),
        'maps',
        'tb3_house_map.yaml'))
    
    # Configuraciones y directorios
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Lanzamiento del robot Turtlebot3 en Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )
    
    # Lanzamiento de la navegación
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
            'autostart': 'true'
        }.items()
    )
    
    # Servidor de mapas
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # Gestor del ciclo de vida
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ],
    )
    
    # AMCL para localización
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('aidguide_04_slam'), 'config', 'amcl_params.yaml'),
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # RViz para visualización
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Nodo de localización personalizado
    aidguide_localization = Node(
        package='aidguide_04_slam',
        executable='localization_node',
        name='aidguide_localization_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Crear y retornar el lanzamiento
    return LaunchDescription([
        # Declaraciones
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar tiempo de simulación'),
        DeclareLaunchArgument(
            'map_file',
            default_value=os.path.join(
                get_package_share_directory('aidguide_04_slam'),
                'maps',
                'tb3_house_map.yaml'),
            description='Ruta completa al archivo de mapa'),
            
        # Nodos y lanzamientos
        gazebo_launch,
        map_server,
        lifecycle_manager,
        amcl,
        rviz,
        aidguide_localization
    ]) 