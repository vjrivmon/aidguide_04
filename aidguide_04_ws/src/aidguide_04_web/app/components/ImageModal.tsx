import { useState, useEffect } from 'react';
import { X, Check, Camera, PaintBucket, Square, Maximize, Circle, FileSearch } from 'lucide-react';
import Image from 'next/image';

// Tipos de transformación disponibles
type TransformationType = 'original' | 'edges' | 'colors' | 'shapes' | 'blobs' | 'canny';

// Props del componente
interface ImageModalProps {
  isOpen: boolean;
  onClose: () => void;
  imagePath: string;
  imageAlt: string;
  detectedDate: string;
}

export default function ImageModal({ isOpen, onClose, imagePath, imageAlt, detectedDate }: ImageModalProps) {
  const [currentTransformation, setCurrentTransformation] = useState<TransformationType>('original');
  const [transformedImagePath, setTransformedImagePath] = useState<string>(imagePath);
  const [isTransformationLoading, setIsTransformationLoading] = useState<boolean>(false);
  const [transformationError, setTransformationError] = useState<string | null>(null);

  // Restaurar a la imagen original cuando se cierra y vuelve a abrir el modal
  useEffect(() => {
    if (isOpen) {
      setCurrentTransformation('original');
      setTransformedImagePath(imagePath);
      setTransformationError(null);
    }
  }, [isOpen, imagePath]);

  // Aplicar transformación a la imagen
  const applyTransformation = async (type: TransformationType) => {
    setIsTransformationLoading(true);
    setTransformationError(null);
    
    try {
      if (type === 'original') {
        setTransformedImagePath(imagePath);
      } else if (type === 'edges' || type === 'colors' || type === 'shapes' || type === 'blobs' || type === 'canny') {
        // Extraer información de la ruta de la imagen original
        const fileName = imagePath.split('/').pop();
        if (!fileName) {
          throw new Error('No se pudo determinar el nombre del archivo');
        }
        
        const response = await fetch('http://192.168.0.17:5000/api/transform', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            filename: fileName,
            transform_type: type === 'colors' ? 'color' : type
          })
        });

        if (!response.ok) {
          throw new Error(`No se pudo procesar la imagen (${type})`);
        }

        const blob = await response.blob();
        const url = URL.createObjectURL(blob);
        setTransformedImagePath(url);
      } else {
        // Para otros tipos de transformación (aún no implementados), usar la imagen original
        setTransformedImagePath(imagePath);
        // Mostrar un mensaje de "no implementado aún"
        alert(`La transformación "${type}" será implementada próximamente.`);
        setIsTransformationLoading(false);
        return;
      }
      
      setCurrentTransformation(type);
    } catch (error) {
      console.error('Error al aplicar transformación:', error);
      setTransformationError(`Error al aplicar la transformación: ${error}`);
      // Revertir a la imagen original en caso de error
      setTransformedImagePath(imagePath);
    } finally {
      setIsTransformationLoading(false);
    }
  };

  // Manejar errores de carga de imagen
  const handleImageError = () => {
    console.warn('Error al cargar imagen transformada:', transformedImagePath);
    
    // Si estamos en modo "edges" (bordes) y hay un error, intentar una ruta alternativa
    if (currentTransformation === 'edges') {
      const fileName = imagePath.split('/').pop();
      if (fileName) {
        // Intentar otra estructura de ruta (puede variar según la estructura real)
        const alternativePath = `/contorno_${fileName}`;
        console.log('Intentando ruta alternativa:', alternativePath);
        setTransformedImagePath(alternativePath);
        return;
      }
    }
    
    // Si no podemos recuperarnos, volver a la imagen original
    setTransformationError('No se pudo cargar la imagen transformada');
    setTransformedImagePath(imagePath);
    setCurrentTransformation('original');
  };

  if (!isOpen) return null;

  // Lista de transformaciones disponibles
  const transformations = [
    { id: 'original', name: 'Original', icon: Camera, available: true },
    { id: 'edges', name: 'Bordes', icon: Square, available: true },
    { id: 'canny', name: 'Canny', icon: FileSearch, available: true },
    { id: 'colors', name: 'Colores', icon: PaintBucket, available: true },
    { id: 'shapes', name: 'Formas', icon: Maximize, available: true },
    { id: 'blobs', name: 'Blobs', icon: Circle, available: true }
  ];

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4 bg-black bg-opacity-70">
      <div className="bg-white rounded-lg overflow-hidden shadow-2xl w-full max-w-6xl h-[80vh] flex flex-col">
        {/* Cabecera */}
        <div className="flex justify-between items-center px-4 py-3 border-b">
          <h3 className="text-lg font-semibold text-gray-900">Visualizador de Imágenes</h3>
          <button 
            onClick={onClose}
            className="text-gray-500 hover:text-gray-700 focus:outline-none"
          >
            <X size={24} />
          </button>
        </div>
        
        {/* Contenido principal */}
        <div className="flex flex-1 overflow-hidden">
          {/* Panel lateral con opciones */}
          <div className="w-64 border-r p-4 bg-gray-50 overflow-y-auto">
            <h4 className="text-sm font-semibold mb-3 text-gray-700">Transformaciones</h4>
            
            {transformations.map((transform) => (
              <button
                key={transform.id}
                onClick={() => applyTransformation(transform.id as TransformationType)}
                disabled={!transform.available || isTransformationLoading}
                className={`flex items-center w-full p-3 mb-2 rounded-lg transition-colors ${
                  currentTransformation === transform.id 
                    ? 'bg-button text-white' 
                    : transform.available 
                      ? 'text-gray-700 hover:bg-gray-200' 
                      : 'text-gray-400 cursor-not-allowed'
                }`}
                title={`${transform.name}${!transform.available ? ' (Próximamente)' : ''}`}
              >
                <transform.icon size={20} className="mr-3" />
                <span className="text-sm">{transform.name}</span>
                {!transform.available && (
                  <span className="ml-auto text-xs">Próx.</span>
                )}
                {currentTransformation === transform.id && (
                  <Check size={16} className="ml-auto" />
                )}
              </button>
            ))}
          </div>
          
          {/* Imagen */}
          <div className="flex-1 bg-gray-50 relative overflow-hidden">
            <div className="absolute inset-0 flex items-center justify-center p-4">
              <div className="relative w-full h-full max-w-full max-h-full">
                {isTransformationLoading ? (
                  <div className="absolute inset-0 flex items-center justify-center">
                    <div className="animate-spin rounded-full h-16 w-16 border-b-2 border-button"></div>
                  </div>
                ) : transformationError ? (
                  <div className="absolute inset-0 flex flex-col items-center justify-center text-red-500 p-4 text-center">
                    <p>{transformationError}</p>
                    <button 
                      onClick={() => applyTransformation('original')}
                      className="mt-4 bg-button text-white px-4 py-2 rounded hover:bg-opacity-90"
                    >
                      Volver a Original
                    </button>
                  </div>
                ) : (
                  <Image 
                    src={transformedImagePath}
                    alt={`${imageAlt} - ${currentTransformation}`}
                    fill
                    className="object-contain"
                    priority
                    sizes="(max-width: 768px) 100vw, 80vw"
                    onError={handleImageError}
                  />
                )}
              </div>
            </div>
            
            {/* Información de la imagen */}
            <div className="absolute bottom-0 left-0 right-0 bg-gradient-to-t from-black to-transparent p-4 text-white">
              <p className="opacity-80 text-sm">
                Detectado: {detectedDate}
                {currentTransformation !== 'original' && (
                  <span className="ml-2">| Transformación: {transformations.find(t => t.id === currentTransformation)?.name}</span>
                )}
              </p>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
} 