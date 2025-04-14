"use client";

import { useState, useEffect, useRef } from "react";
import { MapPin, Navigation, Battery, Activity, Clock, AlertCircle, Wifi, Maximize2, Minimize2 } from "lucide-react";
import { useAuth } from "@/context/auth-context";
import { useRobot } from "@/context/robot-context";
import yaml from "js-yaml";

interface MapData {
  resolution: number;
  origin: [number, number, number];
}

interface Position {
  x: number;
  y: number;
}

export default function FamilyPage() {
  const { user } = useAuth();
  const { batteryPercentage, batteryStatus, estimatedTimeRemaining, isConnected, robotPose } = useRobot();
  const [fromLocation, setFromLocation] = useState<string>("");
  const [toLocation, setToLocation] = useState<string>("");
  const [stopLocation, setStopLocation] = useState<string>("");
  const [cameraSrc, setCameraSrc] = useState<string>("");
  const [isFullscreen, setIsFullscreen] = useState<boolean>(false);
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [mapLoadError, setMapLoadError] = useState<string | null>(null);
  const videoContainerRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const mapImageDataRef = useRef<ImageData | null>(null); // Para almacenar los datos del mapa

  // Cargar metadatos del mapa YAML
  useEffect(() => {
    fetch("/aidguide_04_map.yaml") // Ajusta la ruta según tu proyecto
      .then((response) => {
        if (!response.ok) throw new Error("YAML no encontrado");
        return response.text();
      })
      .then((yamlText) => {
        const parsedData = yaml.load(yamlText) as MapData;
        setMapData(parsedData);
        console.log("YAML cargado:", parsedData);
      })
      .catch((error) => console.error("Error loading YAML:", error));
  }, []);

  const resolution = mapData?.resolution || 0.05;
  const origin = mapData?.origin || [0, 0, 0];

  // Transformar coordenadas ROS a píxeles del mapa
  const rosToMap = (x: number, y: number): Position => {
    if (!canvasRef.current) return { x: 0, y: 0 };
    const pixelX = (x - origin[0]) / resolution;
    const pixelY = canvasRef.current.height - (y - origin[1]) / resolution;
    console.log("Coordenadas ROS:", x, y, "-> Píxeles:", pixelX, pixelY);
    return { x: pixelX, y: pixelY };
  };

  // Cargar y dibujar el mapa PGM una vez al montar el componente
  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");
    if (!canvas || !ctx) return;

    const loadMap = async () => {
      try {
        const response = await fetch("/aidguide_04_map.pgm"); // Ajusta la ruta según tu proyecto
        if (!response.ok) throw new Error(`HTTP error! status: ${response.status}`);
        const buffer = await response.arrayBuffer();
        const data = new Uint8Array(buffer);
        let offset = 0;

        const textDecoder = new TextDecoder();
        let header = "";
        while (offset < data.length && data[offset] !== 10) {
          header += String.fromCharCode(data[offset]);
          offset++;
        }
        offset++;

        if (header !== "P5") throw new Error("Unsupported PGM format. Expected P5.");

        let width = "",
          height = "",
          maxVal = "";
        while (offset < data.length && data[offset] !== 32) {
          width += String.fromCharCode(data[offset]);
          offset++;
        }
        offset++;
        while (offset < data.length && data[offset] !== 10) {
          height += String.fromCharCode(data[offset]);
          offset++;
        }
        offset++;
        while (offset < data.length && data[offset] !== 10) {
          maxVal += String.fromCharCode(data[offset]);
          offset++;
        }
        offset++;

        const imgWidth = parseInt(width);
        const imgHeight = parseInt(height);
        const maxValue = parseInt(maxVal);

        canvas.width = imgWidth;
        canvas.height = imgHeight;

        const pixelData = data.slice(offset);
        const imageData = ctx.createImageData(imgWidth, imgHeight);
        for (let i = 0; i < pixelData.length; i++) {
          const gray = (pixelData[i] / maxValue) * 255;
          imageData.data[i * 4] = gray;
          imageData.data[i * 4 + 1] = gray;
          imageData.data[i * 4 + 2] = gray;
          imageData.data[i * 4 + 3] = 255;
        }
        ctx.putImageData(imageData, 0, 0);
        mapImageDataRef.current = imageData; // Guardamos los datos del mapa
        console.log("Mapa PGM cargado con dimensiones:", imgWidth, imgHeight);
      } catch (error) {
        console.error("Error loading PGM:", error);
        setMapLoadError("No se pudo cargar el mapa PGM.");
      }
    };

    loadMap();
  }, []);

  // Actualizar la posición del robot en el mapa
  useEffect(() => {
    const canvas = canvasRef.current;
    const ctx = canvas?.getContext("2d");
    if (!canvas || !ctx || !mapImageDataRef.current) {
      console.log("No se puede dibujar: canvas, ctx o mapImageData no están listos");
      return;
    }
    const redrawMap = () => {
      const canvas = canvasRef.current;
      const ctx = canvas?.getContext("2d");
      if (!canvas || !ctx || !mapImageDataRef.current) {
        console.log("No se puede dibujar: canvas, ctx o mapImageData no están listos");
        return;
      }
    
      // Redibujar el mapa primero
      ctx.clearRect(0, 0, canvas.width, canvas.height);
    
      // Guardar el contexto antes de la rotación
      ctx.save();
    
      // Cambiar las dimensiones del canvas para adaptarse a la rotación (ancho y alto invertidos)
      canvas.width = canvas.height; // Cambiar el ancho al alto
      canvas.height = canvas.width; // Cambiar el alto al ancho
    
      // Mover el contexto al centro para rotar alrededor del centro de la imagen
      ctx.translate(canvas.width / 2, canvas.height / 2);
    
      // Rotar 90 grados a la izquierda
      ctx.rotate(-Math.PI / 2); // Rotación en sentido antihorario
    
      // Mover el contexto de nuevo al origen para dibujar la imagen correctamente
      ctx.translate(-canvas.height / 2, -canvas.width / 2); // Ajustar el centro
    
      // Dibujar el mapa
      ctx.putImageData(mapImageDataRef.current, 0, 0);
    
      // Restaurar el contexto a su estado original después de la rotación
      ctx.restore();
    
      // Dibujar el robot si la información de la posición está disponible
      if (robotPose) {
        const { x, y } = rosToMap(robotPose.x, robotPose.y);
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, 2 * Math.PI);
        ctx.fillStyle = "green";
        ctx.fill();
        console.log("Robot dibujado en:", x, y);
      } else {
        console.log("robotPose no está disponible aún");
      }
    };
    

    redrawMap();
  }, [robotPose, mapData]); // Dependencias: robotPose y mapData

  // Configuración de la cámara
  useEffect(() => {
    const updateCameraFeed = () => {
      const timestamp = new Date().getTime();
      setCameraSrc(`http://localhost:8080/stream?topic=/camera/image_raw&t=${timestamp}`);
    };
    updateCameraFeed();
    const interval = setInterval(updateCameraFeed, 1000);
    return () => clearInterval(interval);
  }, []);

  const enterFullscreen = () => {
    if (videoContainerRef.current?.requestFullscreen) {
      videoContainerRef.current.requestFullscreen();
      setIsFullscreen(true);
    }
  };

  const exitFullscreen = () => {
    if (document.fullscreenElement && document.exitFullscreen) {
      document.exitFullscreen();
      setIsFullscreen(false);
    }
  };

  const handleStartRoute = () => {
    console.log("Iniciando ruta desde:", fromLocation, "hasta:", toLocation, "con parada en:", stopLocation);
  };

  return (
    <div className="container-custom py-14">
      <div className="text-center mb-12">
        <h1 className="text-4xl md:text-4xl font-bold mb-4">Bienvenido, {user?.nombre}</h1>
        <h2 className="text-2xl text-text">Seguimiento y control del robot de su hija María</h2>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
        <div className="bg-white rounded-lg shadow-md p-6">
          <h2 className="text-2xl font-bold text-button mb-6">Estado del robot de María</h2>
          <div className="space-y-6">
            <div className="bg-gray-50 p-4 rounded-lg">
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center">
                  <Battery className="text-button mr-2" size={20} />
                  <span className="text-text font-medium">Batería</span>
                </div>
                <span className="text-text font-medium">{batteryPercentage}%</span>
              </div>
              <div className="w-full h-2 bg-gray-200 rounded-full">
                <div
                  className={`h-full rounded-full ${
                    batteryPercentage > 60 ? "bg-green-500" : batteryPercentage > 20 ? "bg-yellow-500" : "bg-red-500"
                  }`}
                  style={{ width: `${batteryPercentage}%` }}
                />
              </div>
              <p className="text-sm text-gray-500 mt-2">
                {batteryStatus === "charging"
                  ? `Cargando - ${estimatedTimeRemaining}`
                  : `Tiempo estimado restante: ${estimatedTimeRemaining}`}
              </p>
            </div>
            <div className="bg-gray-50 p-4 rounded-lg">
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center">
                  <Wifi className="text-button mr-2" size={20} />
                  <span className="text-text font-medium">Conexión</span>
                </div>
                <span className={`font-medium ${isConnected ? "text-green-500" : "text-red-500"}`}>
                  {isConnected ? "Connected" : "Not connected"}
                </span>
              </div>
            </div>
            <div className="bg-gray-50 p-4 rounded-lg">
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center">
                  <Activity className="text-button mr-2" size={20} />
                  <span className="text-text font-medium">Estado</span>
                </div>
                <span className="text-green-500 font-medium">Operativo</span>
              </div>
              <div className="flex items-center space-x-2 text-sm text-gray-500">
                <Clock size={14} />
                <span>Última revisión: Hace 2 días</span>
              </div>
            </div>
            <div className="bg-gray-50 p-4 rounded-lg">
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center">
                  <AlertCircle className="text-button mr-2" size={20} />
                  <span className="text-text font-medium">Averías</span>
                </div>
                <span className="text-green-500 font-medium">Ninguna</span>
              </div>
              <p className="text-sm text-gray-500">Todos los sistemas funcionando correctamente</p>
            </div>
          </div>
        </div>

        <div className="md:col-span-2 bg-white rounded-lg shadow-md p-6">
          <div className="flex justify-between items-center mb-6">
            <div>
              <h2 className="text-2xl font-bold text-button">Navegación</h2>
              <p className="text-sm text-gray-500 mt-1">
                Indica la ruta que debe seguir el robot para que María llegue a su destino
              </p>
            </div>
          </div>
          <div className="space-y-6">
            <div>
              <label className="block text-text mb-2">Desde:</label>
              <div className="relative">
                <input
                  type="text"
                  value={fromLocation}
                  onChange={(e) => setFromLocation(e.target.value)}
                  placeholder="Introduce la ubicación de origen"
                  className="w-full p-3 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-button"
                />
                <button
                  onClick={() => setFromLocation("Mi ubicación actual")}
                  className="absolute right-3 top-1/2 transform -translate-y-1/2 text-button hover:text-button/80"
                >
                  <MapPin size={20} />
                </button>
              </div>
            </div>
            <div>
              <div className="flex items-center justify-between mb-2">
                <label className="block text-text">Añadir parada (opcional):</label>
              </div>
              <div className="relative">
                <input
                  type="text"
                  value={stopLocation}
                  onChange={(e) => setStopLocation(e.target.value)}
                  placeholder="¿Quieres hacer una parada en algún punto?"
                  className="w-full p-3 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-button"
                />
                <button
                  onClick={() => setStopLocation("Mi ubicación actual")}
                  className="absolute right-3 top-1/2 transform -translate-y-1/2 text-button hover:text-button/80"
                >
                  <MapPin size={20} />
                </button>
              </div>
            </div>
            <div>
              <label className="block text-text mb-2">Hasta:</label>
              <div className="relative">
                <input
                  type="text"
                  value={toLocation}
                  onChange={(e) => setToLocation(e.target.value)}
                  placeholder="¿A dónde quieres que vaya?"
                  className="w-full p-3 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-button"
                />
                <button
                  onClick={() => setToLocation("Mi ubicación actual")}
                  className="absolute right-3 top-1/2 transform -translate-y-1/2 text-button hover:text-button/80"
                >
                  <MapPin size={20} />
                </button>
              </div>
            </div>
            <div className="flex justify-center pt-4">
              <button
                onClick={handleStartRoute}
                className="flex items-center px-8 py-3 bg-button text-white rounded-lg hover:opacity-90 transition-colors text-lg"
              >
                <Navigation size={24} className="mr-2" />
                Iniciar Ruta
              </button>
            </div>
          </div>
        </div>
      </div>

      <div className="mt-8 grid grid-cols-1 md:grid-cols-2 gap-8">
        <div className="bg-white rounded-lg shadow-md p-6">
          <h2 className="text-2xl font-bold text-button mb-6">Ubicación de María en tiempo real</h2>
          <div className="relative h-[600px] bg-gray-100 rounded-lg">
            {mapLoadError ? (
              <div className="absolute inset-0 flex items-center justify-center text-red-500">
                {mapLoadError}
              </div>
            ) : (
              <div className="absolute inset-0 flex items-center justify-center">
                <canvas ref={canvasRef} className="max-w-full max-h-full" />
              </div>
            )}
          </div>
        </div>
        <div className="bg-white rounded-lg shadow-md p-6">
          <h2 className="text-2xl font-bold text-button mb-6">Cámara del robot en tiempo real</h2>
          <div ref={videoContainerRef} className="relative h-[600px] bg-gray-100 rounded-lg overflow-hidden">
            {cameraSrc ? (
              <div className="relative h-full">
                <img src={cameraSrc} alt="Camera Feed" className="w-full h-full object-cover rounded-lg" />
                <div className="absolute bottom-3 right-3 bg-black/60 text-white px-3 py-1 rounded-full text-xs">
                  Estado: {isConnected ? "Connected" : "Not connected"}
                </div>
                <div className="absolute top-3 right-3 flex gap-2">
                  {!isFullscreen ? (
                    <button
                      onClick={enterFullscreen}
                      className="bg-black/60 text-white p-2 rounded-full hover:bg-black/80 transition-colors"
                      title="Pantalla completa"
                    >
                      <Maximize2 size={18} />
                    </button>
                  ) : (
                    <button
                      onClick={exitFullscreen}
                      className="bg-black/60 text-white p-2 rounded-full hover:bg-black/80 transition-colors"
                      title="Salir de pantalla completa"
                    >
                      <Minimize2 size={18} />
                    </button>
                  )}
                </div>
              </div>
            ) : (
              <div className="absolute inset-0 flex items-center justify-center">
                <p className="text-gray-500">Cargando vídeo de la cámara...</p>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}