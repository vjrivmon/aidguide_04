#!/usr/bin/env python3
"""
Archivo de lanzamiento para probar SLAM con un robot simulado.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Genera la descripción de lanzamiento para probar SLAM con un robot.
    """
    # Parámetros
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Configuraciones y directorios
    pkg_share = get_package_share_directory('aidguide_04_slam')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # Configuración para slam_toolbox
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    
    # Lanzamiento del robot Turtlebot3 en Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )
    
    # Nodo SLAM Toolbox
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
    
    # Nodo de SLAM personalizado
    aidguide_slam_node = Node(
        package='aidguide_04_slam',
        executable='slam_node',
        name='aidguide_slam_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
    
    # Crear y retornar el lanzamiento
    return LaunchDescription([
        # Declaraciones
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar tiempo de simulación'),
            
        # Nodos y lanzamientos
        gazebo_launch,
        slam_toolbox_node,
        aidguide_slam_node,
        rviz
    ]) 