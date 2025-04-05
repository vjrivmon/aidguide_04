"use client";

import { useState, useEffect } from "react";
import { MapPin, Navigation, Battery, Activity, Clock, AlertCircle, Wifi, Signal, Plus, Trash2 } from "lucide-react";
import { useAuth } from "@/context/auth-context";
import ROSLIB from "roslib";

export default function FamilyPage() {
  const { user } = useAuth();
  const [fromLocation, setFromLocation] = useState("");
  const [toLocation, setToLocation] = useState("");
  const [stopLocation, setStopLocation] = useState("");
  const [connectionStatus, setConnectionStatus] = useState("Not connected");
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [cameraSrc, setCameraSrc] = useState("");
  const rosbridgeAddress = "ws://127.0.0.1:9090/"; // Dirección del servidor ROS

  // Conexión automática a ROS y actualización del feed de la cámara
  useEffect(() => {
    console.log("Page loaded");

    // Conectar a ROS
    const rosInstance = new ROSLIB.Ros({
      url: rosbridgeAddress,
    });

    rosInstance.on("connection", () => {
      setRos(rosInstance);
      setConnectionStatus("Connected");
      console.log("Connection to ROSBridge successful");
    });

    rosInstance.on("error", (error) => {
      setConnectionStatus("Connection error");
      console.log("Connection error:", error);
    });

    rosInstance.on("close", () => {
      setRos(null);
      setConnectionStatus("Not connected");
      console.log("Connection to ROSBridge closed");
    });

    // Actualizar el feed de la cámara cada segundo
    const updateCameraFeed = () => {
      const timestamp = new Date().getTime();
      setCameraSrc(`http://0.0.0.0:8080/stream?topic=/camera/image_raw&t=${timestamp}`);
    };
    updateCameraFeed(); // Llamada inicial
    const interval = setInterval(updateCameraFeed, 1000); // Actualiza cada segundo

    // Limpiar al desmontar
    return () => {
      rosInstance.close();
      clearInterval(interval);
      setRos(null);
      setConnectionStatus("Not connected");
    };
  }, []);

  const handleStartRoute = () => {
    console.log("Iniciando ruta desde:", fromLocation, "hasta:", toLocation, "con parada en:", stopLocation);
  };

  return (
    <div className="container-custom py-14">
      {/* Título y subtítulo */}
      <div className="text-center mb-12">
        <h1 className="text-4xl md:text-4xl font-bold mb-4">
          Bienvenido, {user?.name}
        </h1>
        <h2 className="text-2xl text-text">
          Seguimiento y control del robot de su hija María
        </h2>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
        {/* Panel izquierdo - Estado del robot */}
        <div className="bg-white rounded-lg shadow-md p-6">
          <h2 className="text-2xl font-bold text-button mb-6">Estado del robot de María</h2>
          
          <div className="space-y-6">
            {/* Estado de batería */}
            <div className="bg-gray-50 p-4 rounded-lg">
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center">
                  <Battery className="text-button mr-2" size={20} />
                  <span className="text-text font-medium">Batería</span>
                </div>
                <span className="text-text font-medium">75%</span>
              </div>
              <div className="w-full h-2 bg-gray-200 rounded-full">
                <div className="w-3/4 h-full bg-green-500 rounded-full"></div>
              </div>
              <p className="text-sm text-gray-500 mt-2">Tiempo estimado restante: 4 horas</p>
            </div>

            {/* Estado de conexión */}
            <div className="bg-gray-50 p-4 rounded-lg">
              <div className="flex items-center justify-between mb-2">
                <div className="flex items-center">
                  <Wifi className="text-button mr-2" size={20} />
                  <span className="text-text font-medium">Conexión</span>
                </div>
                <span className={`font-medium ${connectionStatus === "Connected" ? "text-green-500" : "text-red-500"}`}>
                  {connectionStatus}
                </span>
              </div>
            </div>

            {/* Estado operativo */}
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

            {/* Estado de averías */}
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

        {/* Panel central - Navegación */}
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
            {/* Origen */}
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

            {/* Parada intermedia */}
            <div>
              <div className="flex items-center justify-between mb-2">
                <label className="block text-text">Añadir parada (opcional):</label>
                <button
                  onClick={() => setStopLocation("")}
                  className="text-red-500 hover:text-red-600 transition-colors"
                  title="Eliminar parada"
                >
                  <Trash2 size={16} />
                </button>
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

            {/* Destino */}
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

            {/* Botón de inicio de ruta */}
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

      {/* Mapa y Vídeo en tiempo real */}
      <div className="mt-8 grid grid-cols-1 md:grid-cols-2 gap-8">
        {/* Mapa en tiempo real */}
        <div className="bg-white rounded-lg shadow-md p-6">
          <h2 className="text-2xl font-bold text-button mb-6">Ubicación de María en tiempo real</h2>
          <div className="relative h-[600px] bg-gray-100 rounded-lg">
            <div className="absolute inset-0 flex items-center justify-center">
              <p className="text-gray-500">Mapa en tiempo real</p>
            </div>
          </div>
        </div>

        {/* Vídeo en tiempo real */}
        <div className="bg-white rounded-lg shadow-md p-6">
          <h2 className="text-2xl font-bold text-button mb-6">Cámara del robot en tiempo real</h2>
          <div className="relative h-[600px] bg-gray-100 rounded-lg">
            {cameraSrc ? (
              <img
                src={cameraSrc}
                alt="Camera Feed"
                className="w-full h-full object-cover rounded-lg"
              />
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