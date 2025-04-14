from launch import LaunchDescription
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
from launch_ros.actions import Node

def generate_launch_description():
    """Función Generate launch description.
    
    Returns:
        Descripción del valor de retorno.
    """
    return LaunchDescription([
        Node(
            package='aidguide_04_weather',
            executable='weather_monitor',
            name='weather_monitor',
            output='screen',
            emulate_tty=True,
        )
    ]) 