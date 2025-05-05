#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Procesador de imágenes con OpenCV para detectar contornos en diferentes categorías de imágenes.

Este script toma imágenes de la carpeta public/senyales y detecta contornos.
Guarda las imágenes procesadas en carpetas específicas dentro de public según su categoría.
"""

import cv2
import os
import numpy as np
import shutil
from pathlib import Path

# Rutas de directorios
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PUBLIC_DIR = os.path.join(BASE_DIR, "aidguide_04_web", "public")
SENYALES_DIR = os.path.join(PUBLIC_DIR, "senyales")

# Categorías y sus directorios de destino
CATEGORIES = {
    "traffic": "señales_trafico",
    "people": "personas",
    "bus": "paradas_bus",
    "crosswalk": "pasos_peatones",
    "construction": "obras",
    "road-closed": "calles_cortadas"
}

def create_directories():
    """
    Crea los directorios necesarios para cada categoría si no existen.
    """
    for category in CATEGORIES.values():
        category_dir = os.path.join(PUBLIC_DIR, category)
        if not os.path.exists(category_dir):
            os.makedirs(category_dir)
            print(f"Directorio creado: {category_dir}")

def detect_contours(image_path, output_path):
    """
    Detecta los contornos en una imagen y guarda la imagen resultante.
    
    Args:
        image_path (str): Ruta a la imagen de origen
        output_path (str): Ruta donde guardar la imagen con contornos
    
    Returns:
        bool: True si se procesó correctamente, False en caso contrario
    """
    try:
        # Cargar la imagen
        img = cv2.imread(image_path)
        if img is None:
            print(f"No se pudo cargar la imagen: {image_path}")
            return False
            
        # Convertir a escala de grises
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Aplicar umbralización binaria
        ret, umbral = cv2.threshold(img_gray, 155, 255, cv2.THRESH_BINARY)
        
        # Encontrar contornos
        contornos, jerarquia = cv2.findContours(umbral, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        # Dibujar contornos en la imagen original
        img_contornos = img.copy()
        cv2.drawContours(img_contornos, contornos, -1, (0, 165, 255), 3)
        
        # Guardar la imagen con contornos
        cv2.imwrite(output_path, img_contornos)
        print(f"Imagen procesada guardada en: {output_path}")
        
        return True
    except Exception as e:
        print(f"Error al procesar la imagen {image_path}: {str(e)}")
        return False

def classify_image(image_path):
    """
    Clasifica la imagen en una de las categorías definidas.
    Esta es una implementación simple que usa el nombre del archivo para determinar la categoría.
    En una implementación real, se podría usar un modelo de clasificación de imágenes.
    
    Args:
        image_path (str): Ruta a la imagen
        
    Returns:
        str: Categoría de la imagen
    """
    # Implementación simple basada en nombres de archivo
    # En un caso real, utilizaríamos un clasificador de imágenes
    filename = os.path.basename(image_path).lower()
    
    if "señal" in filename or "senal" in filename or "senyal" in filename:
        return "traffic"
    elif "persona" in filename or "people" in filename:
        return "people"
    elif "bus" in filename or "parada" in filename:
        return "bus"
    elif "paso" in filename or "peatón" in filename or "peaton" in filename:
        return "crosswalk"
    elif "obra" in filename or "construction" in filename:
        return "construction"
    elif "calle" in filename or "corte" in filename or "cortada" in filename:
        return "road-closed"
    else:
        # Por defecto, categorizar como señales de tráfico
        return "traffic"
        
def process_images():
    """
    Procesa todas las imágenes en la carpeta de origen, detecta contornos
    y las guarda en sus respectivas carpetas según su categoría.
    """
    # Crear directorios si no existen
    create_directories()
    
    # Recorrer todas las imágenes en la carpeta de origen
    for filename in os.listdir(SENYALES_DIR):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
            image_path = os.path.join(SENYALES_DIR, filename)
            
            # Clasificar la imagen
            category = classify_image(image_path)
            category_dir = os.path.join(PUBLIC_DIR, CATEGORIES[category])
            
            # Definir el nombre de archivo de salida
            output_filename = f"contorno_{filename}"
            output_path = os.path.join(category_dir, output_filename)
            
            # Detectar contornos y guardar la imagen
            success = detect_contours(image_path, output_path)
            
            if success:
                print(f"Imagen {filename} procesada y categorizada como {category}")
            else:
                print(f"No se pudo procesar la imagen {filename}")

def main():
    """
    Función principal
    """
    print("Iniciando procesamiento de imágenes...")
    process_images()
    print("Procesamiento de imágenes completado.")

if __name__ == "__main__":
    main() 