#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Función Generate launch description.
    
    Returns:
        Descripción del valor de retorno.
    """
    return LaunchDescription([
        Node(
            package='aidguide_04_nav_punto_a_punto',
            executable='punto_a_punto_node',
            name='punto_a_punto_node',
            output='screen'
        )
    ])