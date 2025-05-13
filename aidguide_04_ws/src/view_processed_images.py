#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Visualizador de imágenes procesadas para ver los contornos detectados.

Este script muestra las imágenes originales y sus versiones procesadas con contornos
detectados lado a lado para facilitar la comparación y verificación de resultados.
"""

import cv2
import os
import numpy as np
import glob
from pathlib import Path

# Rutas de directorios
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PUBLIC_DIR = os.path.join(BASE_DIR, "aidguide_04_web", "public")
SENYALES_DIR = os.path.join(PUBLIC_DIR, "senyales")

# Categorías y sus directorios
CATEGORIES = {
    "señales_trafico": "Señales de Tráfico",
    "personas": "Personas",
    "paradas_bus": "Paradas de Autobús",
    "pasos_peatones": "Pasos de Peatones",
    "obras": "Obras",
    "calles_cortadas": "Calles Cortadas"
}

def resize_image(image, width=None, height=None):
    """
    Redimensiona una imagen manteniendo su proporción.
    
    Args:
        image (numpy.ndarray): Imagen a redimensionar
        width (int, optional): Ancho deseado
        height (int, optional): Altura deseada
        
    Returns:
        numpy.ndarray: Imagen redimensionada
    """
    h, w = image.shape[:2]
    
    if width is None and height is None:
        return image
    
    if width is None:
        aspect_ratio = height / float(h)
        dim = (int(w * aspect_ratio), height)
    else:
        aspect_ratio = width / float(w)
        dim = (width, int(h * aspect_ratio))
        
    return cv2.resize(image, dim, interpolation=cv2.INTER_AREA)

def view_image_pairs():
    """
    Muestra pares de imágenes originales y procesadas con contornos.
    Permite navegar entre imágenes usando teclado.
    """
    # Buscar todas las imágenes procesadas por categoría
    processed_images = []
    
    for category_dir in CATEGORIES.keys():
        full_category_path = os.path.join(PUBLIC_DIR, category_dir)
        if os.path.exists(full_category_path):
            # Buscar imágenes que comienzan con "contorno_"
            contour_images = glob.glob(os.path.join(full_category_path, "contorno_*.jpg")) + \
                            glob.glob(os.path.join(full_category_path, "contorno_*.jpeg")) + \
                            glob.glob(os.path.join(full_category_path, "contorno_*.png"))
            
            for contour_image in contour_images:
                # Obtener el nombre del archivo original
                original_filename = os.path.basename(contour_image).replace("contorno_", "")
                original_image = os.path.join(SENYALES_DIR, original_filename)
                
                if os.path.exists(original_image):
                    category_name = CATEGORIES[category_dir]
                    processed_images.append({
                        "category": category_name,
                        "original": original_image,
                        "contour": contour_image
                    })
    
    if not processed_images:
        print("No se encontraron imágenes procesadas. ¿Has ejecutado primero image_processor.py?")
        return
    
    # Variables para navegación
    current_index = 0
    total_images = len(processed_images)
    
    while True:
        # Obtener información de la imagen actual
        current_image = processed_images[current_index]
        category = current_image["category"]
        original_path = current_image["original"]
        contour_path = current_image["contour"]
        
        # Cargar imágenes
        original = cv2.imread(original_path)
        contour = cv2.imread(contour_path)
        
        if original is None or contour is None:
            print(f"Error al cargar las imágenes: {original_path} o {contour_path}")
            current_index = (current_index + 1) % total_images
            continue
        
        # Redimensionar imágenes al mismo tamaño
        height = min(original.shape[0], contour.shape[0], 600)
        original_resized = resize_image(original, height=height)
        contour_resized = resize_image(contour, height=height)
        
        # Crear imagen combinada
        combined = np.hstack((original_resized, contour_resized))
        
        # Mostrar información
        info_text = f"Categoría: {category} | {current_index + 1}/{total_images} | Original (izq) - Contornos (der)"
        cv2.putText(combined, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(combined, "◄ Anterior (A) | Siguiente (D) ► | ESC para salir", (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Mostrar imagen combinada
        window_name = "Visualizador de Imágenes Procesadas"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, combined)
        
        # Esperar tecla
        key = cv2.waitKey(0) & 0xFF
        
        # Navegación
        if key == 27:  # ESC
            break
        elif key == ord('a') or key == ord('A') or key == 81:  # A o flecha izquierda
            current_index = (current_index - 1) % total_images
        elif key == ord('d') or key == ord('D') or key == 83:  # D o flecha derecha
            current_index = (current_index + 1) % total_images
    
    cv2.destroyAllWindows()

def main():
    """
    Función principal
    """
    print("Iniciando visualizador de imágenes procesadas...")
    view_image_pairs()
    print("Visualizador cerrado.")

if __name__ == "__main__":
    main() 