#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Integrador de imágenes procesadas con la interfaz web de robot-feed.

Este script prepara las imágenes procesadas para que estén disponibles 
en la interfaz web de robot-feed según su categoría.
"""

import os
import json
import glob
import shutil
from datetime import datetime
from pathlib import Path

# Rutas de directorios
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PUBLIC_DIR = os.path.join(BASE_DIR, "aidguide_04_web", "public")
DATA_DIR = os.path.join(BASE_DIR, "aidguide_04_web", "app", "robot-feed", "data")

# Mapeo de directorios de categorías a IDs de categorías en la interfaz web
CATEGORY_MAP = {
    "señales_trafico": "traffic",
    "personas": "people",
    "paradas_bus": "bus",
    "pasos_peatones": "crosswalk",
    "obras": "construction",
    "calles_cortadas": "road-closed"
}

def ensure_data_directory():
    """
    Asegura que el directorio de datos exista para almacenar la información de las imágenes.
    """
    if not os.path.exists(DATA_DIR):
        os.makedirs(DATA_DIR)
        print(f"Directorio de datos creado: {DATA_DIR}")

def get_image_data():
    """
    Recoge información sobre las imágenes procesadas y crea un archivo JSON.
    
    Returns:
        dict: Datos de las imágenes por categoría
    """
    image_data = {}
    
    for category_dir, category_id in CATEGORY_MAP.items():
        full_category_path = os.path.join(PUBLIC_DIR, category_dir)
        if not os.path.exists(full_category_path):
            continue
            
        # Buscar imágenes procesadas con contornos
        contour_images = []
        for ext in ['.jpg', '.jpeg', '.png']:
            contour_images.extend(glob.glob(os.path.join(full_category_path, f"contorno_*{ext}")))
        
        # Recopilar datos de cada imagen
        category_images = []
        for img_path in contour_images:
            # Obtener información del archivo
            filename = os.path.basename(img_path)
            rel_path = os.path.join(category_dir, filename)
            
            # Obtener fecha de modificación como timestamp
            timestamp = os.path.getmtime(img_path)
            date_detected = datetime.fromtimestamp(timestamp).strftime('%d/%m/%Y, %H:%M')
            
            # Añadir datos de la imagen
            category_images.append({
                "path": f"/{rel_path}",
                "detected": date_detected,
                "confidence": 0.95  # Valor ficticio para simular la confianza de detección
            })
        
        # Guardar imágenes de esta categoría
        if category_images:
            image_data[category_id] = category_images
    
    return image_data

def save_image_data(image_data):
    """
    Guarda los datos de las imágenes en archivos JSON.
    
    Args:
        image_data (dict): Datos de las imágenes por categoría
    """
    # Archivo original para compatibilidad con el código existente
    data_file = os.path.join(DATA_DIR, "processed_images.json")
    
    # Nuevo archivo para la API de Next.js
    api_dir = os.path.join(BASE_DIR, "aidguide_04_web", "app", "api", "images.json")
    api_file = os.path.join(api_dir, "data.json")
    
    # Guardar en el directorio original
    os.makedirs(os.path.dirname(data_file), exist_ok=True)
    with open(data_file, 'w', encoding='utf-8') as f:
        json.dump(image_data, f, indent=2, ensure_ascii=False)
    
    # Guardar en el directorio de la API
    os.makedirs(os.path.dirname(api_file), exist_ok=True)
    with open(api_file, 'w', encoding='utf-8') as f:
        json.dump(image_data, f, indent=2, ensure_ascii=False)
    
    print(f"Datos de imágenes guardados en: {data_file}")
    print(f"Datos de imágenes también guardados para la API en: {api_file}")
    
    # Mostrar resumen por categoría
    print("\nResumen de imágenes procesadas:")
    for category_id, images in image_data.items():
        category_name = next((name for dir_name, name in zip(CATEGORY_MAP.keys(), CATEGORY_MAP.values()) if name == category_id), category_id)
        print(f"- {category_name}: {len(images)} imágenes")

def create_example_component():
    """
    Crea un archivo de ejemplo para integrar las imágenes procesadas
    en la interfaz web de robot-feed.
    """
    component_file = os.path.join(BASE_DIR, "aidguide_04_web", "app", "components", "ImageGrid.tsx")
    
    if os.path.exists(component_file):
        print(f"El componente ya existe: {component_file}")
        return
    
    component_code = """
import { useState, useEffect } from 'react';
import Image from 'next/image';

interface ImageItem {
  path: string;
  detected: string;
  confidence: number;
}

interface ImageGridProps {
  categoryId: string | null;
}

export default function ImageGrid({ categoryId }: ImageGridProps) {
  const [images, setImages] = useState<ImageItem[]>([]);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    async function loadImages() {
      setLoading(true);
      try {
        // Cargar datos de imágenes procesadas desde el archivo JSON
        const response = await fetch('/robot-feed/data/processed_images.json');
        const data = await response.json();
        
        // Si la categoría existe y tiene imágenes, mostrarlas
        if (categoryId && data[categoryId]) {
          setImages(data[categoryId]);
        } else {
          setImages([]);
        }
      } catch (error) {
        console.error('Error al cargar imágenes:', error);
        setImages([]);
      }
      setLoading(false);
    }

    if (categoryId && categoryId !== 'live') {
      loadImages();
    } else {
      setImages([]);
      setLoading(false);
    }
  }, [categoryId]);

  if (loading) {
    return (
      <div className="flex items-center justify-center h-48">
        <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-button"></div>
      </div>
    );
  }

  if (images.length === 0) {
    return (
      <div className="text-center py-10">
        <p className="text-lg text-gray-500">
          No hay imágenes disponibles para esta categoría.
        </p>
      </div>
    );
  }

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
      {images.map((image, index) => (
        <div key={index} className="bg-gray-50 p-4 rounded-lg">
          <div className="relative h-48 bg-gray-200 rounded-lg overflow-hidden">
            <Image
              src={image.path}
              alt={`Imagen procesada ${index + 1}`}
              fill
              className="object-cover"
            />
          </div>
          <div className="mt-2 flex justify-between items-center">
            <p className="text-sm text-gray-500">Detectado: {image.detected}</p>
            <p className="text-sm font-medium text-button">
              {Math.round(image.confidence * 100)}% confianza
            </p>
          </div>
        </div>
      ))}
    </div>
  );
}
"""
    
    os.makedirs(os.path.dirname(component_file), exist_ok=True)
    
    with open(component_file, 'w', encoding='utf-8') as f:
        f.write(component_code.strip())
    
    print(f"Componente de ejemplo creado: {component_file}")
    print("Nota: Para integrar completamente este componente, deberás modificar la página robot-feed/page.tsx")
    print("para importar y usar el componente ImageGrid pasándole la categoría seleccionada.")

def main():
    """
    Función principal
    """
    print("Iniciando integración de imágenes procesadas con la interfaz web...")
    
    # Asegurar que existe el directorio de datos
    ensure_data_directory()
    
    # Recopilar datos de imágenes procesadas
    image_data = get_image_data()
    
    if not image_data:
        print("No se encontraron imágenes procesadas. ¿Has ejecutado primero image_processor.py?")
        return
    
    # Guardar datos en archivo JSON
    save_image_data(image_data)
    
    # Crear componente de ejemplo
    create_example_component()
    
    print("\nIntegración completada. Para ver las imágenes en la interfaz web:")
    print("1. Ejecuta el procesador de imágenes si aún no lo has hecho: python image_processor.py")
    print("2. Modifica robot-feed/page.tsx para usar el componente ImageGrid")
    print("3. Reinicia el servidor web si está en ejecución")

if __name__ == "__main__":
    main() 