#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
from launch.conditions import IfCondition

def generate_launch_description():
    """Crear descripción de lanzamiento para monitoreo del robot"""
    
    # Obtener el directorio del paquete
    pkg_dir = get_package_share_directory('aidguide_04_robot_monitoring')
    
    # Definir parámetros de lanzamiento
    auto_start = LaunchConfiguration('auto_start', default='true')
    
    # Declarar argumentos
    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto-start todos los monitores'
    )
    
    # Nodos de monitoreo
    battery_monitor_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='battery_monitor',
        name='battery_monitor',
        output='screen',
        emulate_tty=True
    )
    
    hardware_monitor_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='hardware_monitor',
        name='hardware_monitor',
        output='screen',
        emulate_tty=True
    )
    
    temperature_monitor_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='temperature_monitor',
        name='temperature_monitor',
        output='screen',
        emulate_tty=True
    )
    
    log_monitor_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='log_monitor',
        name='log_monitor',
        output='screen',
        emulate_tty=True
    )
    
    monitoring_dashboard_node = Node(
        package='aidguide_04_robot_monitoring',
        executable='monitoring_dashboard',
        name='monitoring_dashboard',
        output='screen',
        emulate_tty=True
    )
    
    # Mensaje informativo
    startup_info = LogInfo(
        msg=["🚀 Iniciando sistema completo de monitoreo del robot..."]
    )
    
    # Comando para verificar el estado del paquete
    check_package = ExecuteProcess(
        cmd=['ros2', 'pkg', 'list', '|', 'grep', 'aidguide_04_robot_monitoring'],
        shell=True,
        output='screen'
    )
    
    return LaunchDescription([
        declare_auto_start,
        startup_info,
        check_package,
        battery_monitor_node,
        hardware_monitor_node,
        temperature_monitor_node,
        log_monitor_node,
        monitoring_dashboard_node
    ])