"""Archivo de lanzamiento para el nodo de detección de frutas."""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generar la descripción de lanzamiento para el nodo de detección de frutas."""
    return LaunchDescription([
        Node(
            package='aidguide_04_deep_learning',
            executable='fruit_detector',
            name='fruit_detector_node',
            output='screen',
            emulate_tty=True,
            parameters=[]
        )
    ]) 