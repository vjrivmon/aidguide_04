import { NextResponse } from 'next/server';
import fs from 'fs';
import path from 'path';

// Datos de ejemplo de imágenes procesadas por categoría (respaldo)
const mockImageData = {
  "traffic": [
    {
      "path": "/senyales/senyales1.jpg",
      "detected": "05/05/2024, 10:18",
      "confidence": 0.95
    },
    {
      "path": "/senyales/senyales2.jpg",
      "detected": "05/05/2024, 10:19",
      "confidence": 0.92
    },
    {
      "path": "/senyales/senyales4.jpg",
      "detected": "05/05/2024, 10:20",
      "confidence": 0.93
    }
  ],
  "people": [
    {
      "path": "/senyales/senyales4.jpg", // Usar imagen existente para demostración
      "detected": "05/05/2024, 11:30",
      "confidence": 0.88
    }
  ],
  "bus": [
    {
      "path": "/placeholder.jpg",
      "detected": "05/05/2024, 12:15",
      "confidence": 0.85
    }
  ],
  "crosswalk": [
    {
      "path": "/placeholder.jpg",
      "detected": "05/05/2024, 12:45",
      "confidence": 0.87
    }
  ],
  "construction": [
    {
      "path": "/placeholder.jpg",
      "detected": "05/05/2024, 13:20",
      "confidence": 0.82
    }
  ],
  "road-closed": [
    {
      "path": "/placeholder.jpg",
      "detected": "05/05/2024, 14:05",
      "confidence": 0.91
    }
  ]
};

// Función que maneja las solicitudes GET a esta ruta de API
export async function GET() {
  try {
    // Intenta leer el archivo de datos generado por el script de procesamiento
    const dataFilePath = path.join(process.cwd(), 'app', 'api', 'images.json', 'data.json');
    
    let imageData;
    
    if (fs.existsSync(dataFilePath)) {
      // Si el archivo existe, leer sus datos
      const fileContent = fs.readFileSync(dataFilePath, 'utf8');
      imageData = JSON.parse(fileContent);
      console.log('Usando datos de imágenes procesadas del archivo');
    } else {
      // Si el archivo no existe, usar datos de ejemplo
      imageData = mockImageData;
      console.log('Archivo de datos no encontrado, usando datos de ejemplo');
    }
    
    // Devolver los datos JSON con los encabezados adecuados
    return NextResponse.json(imageData, {
      status: 200,
      headers: {
        'Content-Type': 'application/json',
        'Cache-Control': 'no-store, max-age=0'
      }
    });
  } catch (error) {
    console.error('Error al procesar datos de imágenes:', error);
    
    // En caso de error, devolver los datos de ejemplo
    return NextResponse.json(mockImageData, {
      status: 200,
      headers: {
        'Content-Type': 'application/json',
        'Cache-Control': 'no-store, max-age=0'
      }
    });
  }
} 