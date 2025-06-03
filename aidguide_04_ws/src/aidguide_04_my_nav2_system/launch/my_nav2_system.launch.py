import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
from launch_ros.actions import Node

def generate_launch_description():
    """
    Genera una descripción de lanzamiento (LaunchDescription) para iniciar los nodos
    necesarios del sistema de navegación Nav2, incluyendo el mapa, AMCL, RViz, y otros
    componentes personalizados como el nodo de punto inicial y el seguidor de waypoints.

    Returns:
        LaunchDescription: Objeto que contiene la configuración del lanzamiento.
    """

    nav2_yaml = os.path.join(get_package_share_directory('aidguide_04_my_nav2_system'), 'config', 'my_nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('aidguide_04_my_nav2_system'), 'config', 'aidguide_04_map.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('aidguide_04_my_nav2_system'), 'config', 'aidguide_config_robot.rviz')

    return LaunchDescription([
        Node(
            package = 'nav2_map_server',
            executable = 'map_server',
            name = 'map_server',
            output = 'screen',
            parameters=[nav2_yaml,
                        {'yaml_filename':map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names':['map_server', 'amcl', 'planner_server', 'controller_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower']}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        Node(
            package='aidguide_04_my_nav2_system',
            executable='punto_inicial',
            name='punto_inicial',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),

    ])