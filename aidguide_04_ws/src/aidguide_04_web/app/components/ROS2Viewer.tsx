import { useState, useEffect } from 'react';
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs";
import { Play, Square, Refresh, Image as ImageIcon, ImageDown } from 'lucide-react';
import Image from 'next/image';

// Tipos de transformación disponibles para las imágenes de ROS2
type TransformationType = 'original' | 'edges' | 'colors' | 'shapes' | 'blobs' | 'canny';

export default function ROS2Viewer() {
  const [isRunning, setIsRunning] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [images, setImages] = useState<string[]>([]);
  const [selectedImage, setSelectedImage] = useState<string | null>(null);
  const [transformedImage, setTransformedImage] = useState<string | null>(null);
  const [currentTransformation, setCurrentTransformation] = useState<TransformationType>('original');

  // Cargar la lista de imágenes disponibles
  const fetchImages = async () => {
    try {
      setIsLoading(true);
      const response = await fetch('/api/ros2/images');
      
      if (!response.ok) {
        throw new Error('No se pudieron cargar las imágenes de ROS2');
      }
      
      const data = await response.json();
      
      if (data.success && data.images && Array.isArray(data.images)) {
        setImages(data.images);
        // Seleccionar la primera imagen si hay alguna disponible
        if (data.images.length > 0 && !selectedImage) {
          setSelectedImage(data.images[0]);
        }
      } else {
        setImages([]);
      }
      
      setError(null);
    } catch (err) {
      setError('Error al cargar imágenes: ' + (err instanceof Error ? err.message : String(err)));
      console.error('Error fetching ROS2 images:', err);
    } finally {
      setIsLoading(false);
    }
  };
  
  // Obtener la última imagen disponible
  const fetchLatestImage = async () => {
    try {
      setIsLoading(true);
      const response = await fetch('/api/ros2/latest_image');
      
      if (!response.ok) {
        throw new Error('No se pudo obtener la última imagen');
      }
      
      // La respuesta es directamente la imagen
      const blob = await response.blob();
      const url = URL.createObjectURL(blob);
      
      // Guardar la URL de la imagen y refrescar la lista
      setTransformedImage(url);
      fetchImages();
      
      setError(null);
    } catch (err) {
      setError('Error al obtener última imagen: ' + (err instanceof Error ? err.message : String(err)));
      console.error('Error fetching latest ROS2 image:', err);
    } finally {
      setIsLoading(false);
    }
  };
  
  // Iniciar ROS2 y Gazebo
  const startROS2 = async () => {
    try {
      setIsLoading(true);
      setError(null);
      
      const response = await fetch('/api/ros2/start', {
        method: 'POST'
      });
      
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'No se pudo iniciar ROS2 y Gazebo');
      }
      
      setIsRunning(true);
      
      // Esperar un poco para que el sistema se inicie y luego cargar imágenes
      setTimeout(() => {
        fetchImages();
      }, 5000);
      
    } catch (err) {
      setError('Error al iniciar ROS2: ' + (err instanceof Error ? err.message : String(err)));
      console.error('Error starting ROS2:', err);
    } finally {
      setIsLoading(false);
    }
  };
  
  // Detener ROS2 y Gazebo
  const stopROS2 = async () => {
    try {
      setIsLoading(true);
      setError(null);
      
      const response = await fetch('/api/ros2/stop', {
        method: 'POST'
      });
      
      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.error || 'No se pudo detener ROS2 y Gazebo');
      }
      
      setIsRunning(false);
      
    } catch (err) {
      setError('Error al detener ROS2: ' + (err instanceof Error ? err.message : String(err)));
      console.error('Error stopping ROS2:', err);
    } finally {
      setIsLoading(false);
    }
  };
  
  // Aplicar transformación a una imagen
  const applyTransformation = async (imageName: string, type: TransformationType) => {
    if (!imageName) return;
    
    try {
      setIsLoading(true);
      setError(null);
      
      if (type === 'original') {
        // Para la imagen original, simplemente construimos la URL
        setTransformedImage(`/ros2_images/${imageName}`);
        setCurrentTransformation('original');
        setIsLoading(false);
        return;
      }
      
      const response = await fetch('/api/ros2/transform', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          filename: imageName,
          transform_type: type === 'colors' ? 'color' : type
        })
      });
      
      if (!response.ok) {
        throw new Error(`No se pudo procesar la imagen (${type})`);
      }
      
      const blob = await response.blob();
      const url = URL.createObjectURL(blob);
      
      setTransformedImage(url);
      setCurrentTransformation(type);
      
    } catch (err) {
      setError('Error al aplicar transformación: ' + (err instanceof Error ? err.message : String(err)));
      console.error('Error applying transformation:', err);
    } finally {
      setIsLoading(false);
    }
  };
  
  // Cargar la lista de imágenes al montar el componente
  useEffect(() => {
    fetchImages();
    
    // Configurar refrescos periódicos de la lista de imágenes si ROS2 está en ejecución
    let interval: NodeJS.Timeout | null = null;
    
    if (isRunning) {
      interval = setInterval(() => {
        fetchImages();
      }, 10000); // Refrescar cada 10 segundos
    }
    
    return () => {
      if (interval) clearInterval(interval);
    };
  }, [isRunning]);
  
  // Aplicar la transformación cuando cambia la imagen seleccionada
  useEffect(() => {
    if (selectedImage) {
      applyTransformation(selectedImage, currentTransformation);
    }
  }, [selectedImage]);
  
  return (
    <Card className="w-full">
      <CardHeader>
        <CardTitle>Visor de ROS2 Gazebo</CardTitle>
        <CardDescription>
          Control y procesamiento de imágenes de simulación ROS2
        </CardDescription>
      </CardHeader>
      
      <CardContent>
        <div className="mb-6 flex items-center justify-between">
          <div className="space-x-2">
            <Button 
              variant="default" 
              onClick={startROS2} 
              disabled={isRunning || isLoading}
            >
              <Play className="mr-2 h-4 w-4" />
              Iniciar ROS2 y Gazebo
            </Button>
            
            <Button 
              variant="destructive" 
              onClick={stopROS2} 
              disabled={!isRunning || isLoading}
            >
              <Square className="mr-2 h-4 w-4" />
              Detener ROS2 y Gazebo
            </Button>
          </div>
          
          <Button 
            variant="outline" 
            onClick={fetchImages} 
            disabled={isLoading}
          >
            <Refresh className="mr-2 h-4 w-4" />
            Refrescar imágenes
          </Button>
        </div>
        
        {error && (
          <div className="mb-4 p-3 bg-red-100 text-red-800 rounded-md">
            {error}
          </div>
        )}
        
        <div className="grid grid-cols-1 md:grid-cols-4 gap-4">
          {/* Lista de imágenes disponibles */}
          <div className="md:col-span-1 border rounded-md p-2 h-[400px] overflow-y-auto">
            <h3 className="font-medium mb-2">Imágenes disponibles:</h3>
            
            {isLoading && images.length === 0 ? (
              <div className="flex items-center justify-center h-32">
                <div className="animate-spin rounded-full h-8 w-8 border-b-2 border-primary"></div>
              </div>
            ) : images.length === 0 ? (
              <p className="text-sm text-muted-foreground">No hay imágenes disponibles</p>
            ) : (
              <div className="space-y-1">
                {images.map((image) => (
                  <div 
                    key={image}
                    className={`p-2 rounded-md text-sm flex items-center cursor-pointer
                      ${selectedImage === image ? 'bg-primary text-primary-foreground' : 'hover:bg-secondary'}
                    `}
                    onClick={() => setSelectedImage(image)}
                  >
                    <ImageIcon className="h-4 w-4 mr-2" />
                    <span className="truncate">{image}</span>
                  </div>
                ))}
              </div>
            )}
            
            <div className="mt-4">
              <Button 
                variant="outline" 
                size="sm" 
                className="w-full"
                onClick={fetchLatestImage}
                disabled={isLoading || !isRunning}
              >
                <ImageDown className="mr-2 h-4 w-4" />
                Última imagen
              </Button>
            </div>
          </div>
          
          {/* Visualizador de imagen y transformaciones */}
          <div className="md:col-span-3 border rounded-md p-2">
            <Tabs defaultValue="view" className="w-full">
              <TabsList className="mb-4">
                <TabsTrigger value="view">Vista</TabsTrigger>
                <TabsTrigger value="transform">Transformaciones</TabsTrigger>
              </TabsList>
              
              <TabsContent value="view" className="h-[350px] relative">
                {isLoading ? (
                  <div className="flex items-center justify-center h-full">
                    <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary"></div>
                  </div>
                ) : transformedImage ? (
                  <div className="relative h-full w-full">
                    <Image 
                      src={transformedImage}
                      alt="Imagen de ROS2"
                      fill
                      className="object-contain"
                    />
                  </div>
                ) : (
                  <div className="flex items-center justify-center h-full bg-muted/20">
                    <p className="text-muted-foreground">Selecciona una imagen para visualizar</p>
                  </div>
                )}
              </TabsContent>
              
              <TabsContent value="transform">
                <div className="grid grid-cols-3 gap-2">
                  {/* Botones de transformación */}
                  <Button 
                    variant={currentTransformation === 'original' ? 'default' : 'outline'}
                    className="h-20"
                    onClick={() => selectedImage && applyTransformation(selectedImage, 'original')}
                    disabled={isLoading || !selectedImage}
                  >
                    Original
                  </Button>
                  
                  <Button 
                    variant={currentTransformation === 'edges' ? 'default' : 'outline'}
                    className="h-20"
                    onClick={() => selectedImage && applyTransformation(selectedImage, 'edges')}
                    disabled={isLoading || !selectedImage}
                  >
                    Bordes
                  </Button>
                  
                  <Button 
                    variant={currentTransformation === 'canny' ? 'default' : 'outline'}
                    className="h-20"
                    onClick={() => selectedImage && applyTransformation(selectedImage, 'canny')}
                    disabled={isLoading || !selectedImage}
                  >
                    Canny
                  </Button>
                  
                  <Button 
                    variant={currentTransformation === 'colors' ? 'default' : 'outline'}
                    className="h-20"
                    onClick={() => selectedImage && applyTransformation(selectedImage, 'colors')}
                    disabled={isLoading || !selectedImage}
                  >
                    Colores
                  </Button>
                  
                  <Button 
                    variant={currentTransformation === 'shapes' ? 'default' : 'outline'}
                    className="h-20"
                    onClick={() => selectedImage && applyTransformation(selectedImage, 'shapes')}
                    disabled={isLoading || !selectedImage}
                  >
                    Formas
                  </Button>
                  
                  <Button 
                    variant={currentTransformation === 'blobs' ? 'default' : 'outline'}
                    className="h-20"
                    onClick={() => selectedImage && applyTransformation(selectedImage, 'blobs')}
                    disabled={isLoading || !selectedImage}
                  >
                    Blobs
                  </Button>
                </div>
                
                <div className="mt-4 text-sm text-muted-foreground">
                  {selectedImage && (
                    <p>Imagen actual: {selectedImage} | Transformación: {currentTransformation}</p>
                  )}
                </div>
              </TabsContent>
            </Tabs>
          </div>
        </div>
      </CardContent>
      
      <CardFooter className="flex justify-between">
        <p className="text-sm text-muted-foreground">
          Estado: {isRunning ? 'ROS2 y Gazebo en ejecución' : 'Detenido'}
        </p>
      </CardFooter>
    </Card>
  );
} 