#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script principal para ejecutar toda la cadena de procesamiento de imágenes.

Este script ejecuta automáticamente:
1. Procesamiento de imágenes para detectar contornos
2. Integración con la interfaz web
3. Visualización de resultados
"""

import os
import subprocess
import time
import sys

def print_header(title):
    """
    Imprime un encabezado formateado para mejorar la legibilidad.
    
    Args:
        title (str): Título del encabezado
    """
    terminal_width = 80
    print("\n" + "=" * terminal_width)
    print(f"{title.center(terminal_width)}")
    print("=" * terminal_width + "\n")

def run_script(script_name):
    """
    Ejecuta un script Python y espera a que termine.
    
    Args:
        script_name (str): Nombre del script a ejecutar
        
    Returns:
        bool: True si la ejecución fue exitosa, False en caso contrario
    """
    print_header(f"Ejecutando: {script_name}")
    
    try:
        result = subprocess.run([sys.executable, script_name], 
                                check=True, 
                                stdout=subprocess.PIPE, 
                                stderr=subprocess.PIPE,
                                text=True)
        print(result.stdout)
        if result.stderr:
            print("ERRORES/ADVERTENCIAS:")
            print(result.stderr)
        return True
    except subprocess.CalledProcessError as e:
        print(f"ERROR al ejecutar {script_name}:")
        print(e.stderr)
        return False

def main():
    """
    Función principal que ejecuta toda la cadena de procesamiento.
    """
    # Obtener directorio actual
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Scripts a ejecutar en orden
    scripts = [
        os.path.join(current_dir, "image_processor.py"),
        os.path.join(current_dir, "integrate_processed_images.py")
    ]
    
    # Comprobar si todos los scripts existen
    missing_scripts = [s for s in scripts if not os.path.exists(s)]
    if missing_scripts:
        print("ERROR: No se encontraron los siguientes scripts:")
        for script in missing_scripts:
            print(f"  - {script}")
        return
    
    print_header("PROCESAMIENTO DE IMÁGENES CON OPENCV - DETECTOR DE CONTORNOS")
    
    # Ejecutar cada script en secuencia
    success = True
    for script in scripts:
        if not run_script(script):
            success = False
            break
    
    if success:
        print_header("PROCESAMIENTO COMPLETADO CORRECTAMENTE")
        print("Para ver las imágenes procesadas, puedes:")
        print("1. Ejecutar el visualizador de imágenes:")
        print(f"   {sys.executable} {os.path.join(current_dir, 'view_processed_images.py')}")
        print("\n2. Ver las imágenes en la interfaz web:")
        print("   - Asegúrate de que el servidor web esté en ejecución")
        print("   - Navega a la sección 'Imágenes captadas por el robot'")
        print("   - Selecciona una categoría del panel izquierdo")
    else:
        print_header("PROCESAMIENTO INCOMPLETO")
        print("Se encontraron errores durante el procesamiento. Revisa los mensajes anteriores.")

if __name__ == "__main__":
    main() 