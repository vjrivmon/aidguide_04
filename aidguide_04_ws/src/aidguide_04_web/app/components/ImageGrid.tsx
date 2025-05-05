import { useState, useEffect } from 'react';
import Image from 'next/image';
import { Search, ZoomIn } from 'lucide-react';
import ImageModal from './ImageModal';

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
  const [error, setError] = useState<string | null>(null);
  const [modalOpen, setModalOpen] = useState(false);
  const [selectedImage, setSelectedImage] = useState<ImageItem | null>(null);

  useEffect(() => {
    async function loadImages() {
      setLoading(true);
      setError(null);
      
      try {
        // Ruta relativa desde la raíz pública para Next.js
        // Cambiamos cómo se construye la URL para asegurar que sea correcta
        const jsonPath = '/api/images.json'; // Cambio a una ruta de API simulada
        
        console.log('Intentando cargar datos desde:', jsonPath);
        
        // Usar un objeto de solicitud para controlar mejor las cabeceras y opciones
        const requestOptions = {
          method: 'GET',
          headers: { 'Content-Type': 'application/json', 'Cache-Control': 'no-cache' }
        };
        
        const response = await fetch(jsonPath, requestOptions);
        
        if (!response.ok) {
          throw new Error(`Error HTTP: ${response.status} ${response.statusText}`);
        }
        
        // Verificar que el contenido es JSON antes de parsear
        const contentType = response.headers.get('content-type');
        if (!contentType || !contentType.includes('application/json')) {
          console.warn('Respuesta no es JSON:', contentType);
          // Intento de carga de datos simulados como solución alternativa
          setImages(getMockData(categoryId));
          return;
        }
        
        const data = await response.json();
        
        // Verificar si la categoría existe en los datos
        if (categoryId && data && data[categoryId]) {
          setImages(data[categoryId]);
        } else {
          console.log('Categoría no encontrada en datos:', categoryId);
          // Usar datos simulados como alternativa
          setImages(getMockData(categoryId));
        }
      } catch (error) {
        console.error('Error al cargar imágenes:', error);
        setError('No se pudieron cargar las imágenes. Usando datos de ejemplo.');
        // Cargar datos de ejemplo en caso de error
        setImages(getMockData(categoryId));
      } finally {
        setLoading(false);
      }
    }

    // Función para obtener datos simulados por categoría
    function getMockData(category: string | null): ImageItem[] {
      // Si no hay categoría o es 'live', no mostrar nada
      if (!category || category === 'live') return [];
      
      // Datos simulados por categoría
      const mockImages: Record<string, ImageItem[]> = {
        'traffic': [
          { path: '/senyales/senyales1.jpg', detected: 'Hoy, 12:30', confidence: 0.95 },
          { path: '/senyales/senyales2.jpg', detected: 'Hoy, 12:35', confidence: 0.92 }
        ],
        'people': [
          { path: '/senyales/senyales4.jpg', detected: 'Hoy, 12:40', confidence: 0.91 }
        ],
        'bus': [
          { path: '/placeholder.jpg', detected: 'Hoy, 12:45', confidence: 0.88 }
        ],
        'crosswalk': [
          { path: '/placeholder.jpg', detected: 'Hoy, 12:50', confidence: 0.87 }
        ],
        'construction': [
          { path: '/placeholder.jpg', detected: 'Hoy, 12:55', confidence: 0.89 }
        ],
        'road-closed': [
          { path: '/placeholder.jpg', detected: 'Hoy, 13:00', confidence: 0.86 }
        ]
      };
      
      return mockImages[category] || [];
    }

    if (categoryId && categoryId !== 'live') {
      loadImages();
    } else {
      setImages([]);
      setLoading(false);
    }
  }, [categoryId]);

  // Función para abrir el modal con la imagen seleccionada
  const openImageModal = (image: ImageItem) => {
    setSelectedImage(image);
    setModalOpen(true);
  };

  // Función para cerrar el modal
  const closeImageModal = () => {
    setModalOpen(false);
    setSelectedImage(null);
  };

  if (loading) {
    return (
      <div className="flex items-center justify-center h-48">
        <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-button"></div>
      </div>
    );
  }

  if (error) {
    console.warn(error);
    // Seguimos mostrando imágenes incluso si hay error, porque usamos datos simulados
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
    <>
      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        {images.map((image, index) => (
          <div key={index} className="bg-gray-50 p-4 rounded-lg">
            <div 
              className="relative h-48 bg-gray-200 rounded-lg overflow-hidden group cursor-pointer"
              onClick={() => openImageModal(image)}
            >
              <Image
                src={image.path}
                alt={`Imagen procesada ${index + 1}`}
                fill
                className="object-cover transition-transform group-hover:scale-105"
                onError={(e) => {
                  // Fallback en caso de error al cargar la imagen
                  const target = e.target as HTMLImageElement;
                  target.src = '/placeholder.jpg';
                  console.warn('Error al cargar imagen:', image.path);
                }}
              />
              {/* Overlay al hacer hover */}
              <div className="absolute inset-0 bg-black bg-opacity-0 group-hover:bg-opacity-30 transition-opacity flex items-center justify-center opacity-0 group-hover:opacity-100">
                <button className="bg-white bg-opacity-90 p-2 rounded-full">
                  <ZoomIn size={24} className="text-button" />
                </button>
              </div>
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

      {/* Modal para mostrar la imagen ampliada */}
      {selectedImage && (
        <ImageModal
          isOpen={modalOpen}
          onClose={closeImageModal}
          imagePath={selectedImage.path}
          imageAlt={`Imagen detectada`}
          detectedDate={selectedImage.detected}
        />
      )}
    </>
  );
}