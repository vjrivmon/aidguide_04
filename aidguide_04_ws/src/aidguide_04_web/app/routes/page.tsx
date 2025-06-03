"use client"

import { useState, useEffect, useRef } from "react"
import { MapPin, Navigation, Clock, RotateCw, Wifi, WifiOff, AlertTriangle } from "lucide-react"
import dynamic from "next/dynamic"
import ROSService from "@/services/ros-service"

// IMPORTES NECESARIOS PARA EL MAPA (se agregó para la funcionalidad del mapa)
import yaml from "js-yaml"
import { useRobot } from "@/context/robot-context"

// Declaración de la interfaz MapData (puedes moverla a otro archivo e importarla si lo prefieres)
interface MapData {
  resolution: number
  origin: [number, number, number]
}

// ===================== INICIO DEL MAPA - Componente RobotMap =====================
function RobotMap({ hasAnnouncedArrival, setHasAnnouncedArrival }: { hasAnnouncedArrival: boolean; setHasAnnouncedArrival: React.Dispatch<React.SetStateAction<boolean>> }) {
  const { robotPose } = useRobot() // Se asume que robotPose viene del contexto useRobot

  // Se tipan los estados para evitar errores en la asignación
  const [mapData, setMapData] = useState<MapData | null>(null)
  const [mapLoadError, setMapLoadError] = useState<string | null>(null)
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const mapImageDataRef = useRef<ImageData | null>(null) // Para almacenar los datos del mapa

  // Cargar metadatos del mapa YAML
  useEffect(() => {
    fetch("/aidguide_04_map.yaml") // Ajusta la ruta según tu proyecto
      .then((response) => {
        if (!response.ok) throw new Error("YAML no encontrado")
        return response.text()
      })
      .then((yamlText) => {
        const parsedData = yaml.load(yamlText) as MapData
        setMapData(parsedData)
        console.log("YAML cargado:", parsedData)
      })
      .catch((error) => {
        console.error("Error loading YAML:", error)
        setMapLoadError("Error cargando el YAML del mapa.")
      })
  }, [])

  // Se utilizan valores por defecto en caso de que mapData sea null
  const resolution = mapData?.resolution || 0.05
  const origin = mapData?.origin || [0, 0, 0]

  // Convertir coordenadas ROS a píxeles del mapa
  const rosToMap = (x: number, y: number) => {
    const canvas = canvasRef.current
    if (!canvas) return { x: 0, y: 0 }
    const pixelX = (x - origin[0]) / resolution
    const pixelY = canvas.height - (y - origin[1]) / resolution
    console.log("Coordenadas ROS:", x, y, "-> Píxeles:", pixelX, pixelY)
    return { x: pixelX, y: pixelY }
  }

  // Cargar y dibujar el mapa PGM una sola vez al montar el componente
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext("2d")
    if (!ctx) return

    const loadMap = async () => {
      try {
        const response = await fetch("/aidguide_04_map.pgm") // Ajusta la ruta según tu proyecto
        if (!response.ok)
          throw new Error(`HTTP error! status: ${response.status}`)
        const buffer = await response.arrayBuffer()
        const data = new Uint8Array(buffer)
        let offset = 0

        // Leer el header
        const textDecoder = new TextDecoder()
        let header = ""
        while (offset < data.length && data[offset] !== 10) { // 10 es '\n'
          header += String.fromCharCode(data[offset])
          offset++
        }
        offset++ // Saltar el salto de línea

        if (header !== "P5")
          throw new Error("Unsupported PGM format. Expected P5.")

        let width = "", height = "", maxVal = ""
        while (offset < data.length && data[offset] !== 32) { // 32 es ' '
          width += String.fromCharCode(data[offset])
          offset++
        }
        offset++ // Saltar el espacio
        while (offset < data.length && data[offset] !== 10) {
          height += String.fromCharCode(data[offset])
          offset++
        }
        offset++ // Saltar el salto de línea
        while (offset < data.length && data[offset] !== 10) {
          maxVal += String.fromCharCode(data[offset])
          offset++
        }
        offset++ // Saltar el salto de línea

        const imgWidth = parseInt(width)
        const imgHeight = parseInt(height)
        const maxValue = parseInt(maxVal)

        // Asignar dimensiones al canvas (se hace una sola vez)
        canvas.width = imgWidth
        canvas.height = imgHeight

        const pixelData = data.slice(offset)
        const imageData = ctx.createImageData(imgWidth, imgHeight)
        for (let i = 0; i < pixelData.length; i++) {
          const gray = (pixelData[i] / maxValue) * 255
          imageData.data[i * 4] = gray
          imageData.data[i * 4 + 1] = gray
          imageData.data[i * 4 + 2] = gray
          imageData.data[i * 4 + 3] = 255
        }
        ctx.putImageData(imageData, 0, 0)
        mapImageDataRef.current = imageData // Guardamos los datos del mapa
        console.log("Mapa PGM cargado con dimensiones:", imgWidth, imgHeight)
      } catch (error) {
        console.error("Error loading PGM:", error)
        setMapLoadError("No se pudo cargar el mapa PGM.")
      }
    }

    loadMap()
  }, [mapData])

  // Actualizar la posición del robot en el mapa sin modificar las dimensiones del canvas
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext("2d")
    if (!ctx || !mapImageDataRef.current) {
      console.log("No se puede dibujar: canvas, ctx o mapImageData no están listos")
      return
    }
    // Usamos las dimensiones actuales del canvas (ya definidas en loadMap)
    const stableWidth = canvas.width
    const stableHeight = canvas.height

    const redrawMap = () => {
      // Limpiar el canvas usando dimensiones estables
      ctx.clearRect(0, 0, stableWidth, stableHeight)

      // Guardar el contexto y aplicar la transformación para rotar la imagen
      ctx.save()
      ctx.translate(stableWidth / 2, stableHeight / 2)
      ctx.rotate(-Math.PI / 2)
      ctx.translate(-stableWidth / 2, -stableHeight / 2)
      ctx.putImageData(mapImageDataRef.current!, 0, 0)
      ctx.restore()

      // Dibujar el robot si la información está disponible
      if (robotPose) {
        const { x: robotPixelX, y: robotPixelY } = rosToMap(robotPose.x, robotPose.y)
        ctx.beginPath()
        ctx.arc(robotPixelX, robotPixelY, 5, 0, 2 * Math.PI)
        ctx.fillStyle = "green"
        ctx.fill()
        console.log("Robot dibujado en:", robotPixelX, robotPixelY)

        // Comprobar si ha llegado al destino
        if (
          !hasAnnouncedArrival &&
          Math.abs(robotPixelX - 201.17288289548898) < 5 &&
          Math.abs(robotPixelY - 142.0169482641609) < 5
        ) {
          console.log("Llegada detectada!")
          setHasAnnouncedArrival(true)
          const utterance = new SpeechSynthesisUtterance("Has llegado a tu destino")
          utterance.lang = "es-ES"
          
          // Asegurar que las voces estén cargadas y seleccionar una si es posible
          const voices = window.speechSynthesis.getVoices()
          if (voices.length > 0) {
            let spanishVoice = voices.find(voice => voice.lang === 'es-ES')
            if (spanishVoice) {
              utterance.voice = spanishVoice
              console.log(`[RobotMap] Using voice for arrival: ${spanishVoice.name}`)
            }
          } else {
            console.warn("[RobotMap] No speech synthesis voices loaded for arrival. Using default.")
          }
          
          utterance.onerror = (event) => {
            if (event.error === 'interrupted') {
              console.log("[RobotMap] Arrival speech interrupted.")
            } else {
              console.error("[RobotMap] Arrival SpeechSynthesisUtterance error:", event.error, event)
            }
          }
          
          if (window.speechSynthesis) {
            if (window.speechSynthesis.state === 'suspended') {
              window.speechSynthesis.resume()
            }
            window.speechSynthesis.cancel() // Cancelar cualquier anuncio anterior (por si acaso)
            window.speechSynthesis.speak(utterance)
          }
        }
      } else {
        console.log("robotPose no está disponible")
      }
    }

    redrawMap()
  }, [robotPose, mapData, hasAnnouncedArrival, setHasAnnouncedArrival])

  return (
    // Se centra el canvas con flexbox
    <div className="h-full w-full relative flex items-center justify-center">
      {mapLoadError ? (
        <div className="absolute inset-0 flex items-center justify-center text-red-500">
          {mapLoadError}
        </div>
      ) : (
        <canvas ref={canvasRef} className="max-w-full max-h-full" />
      )}
    </div>
  )
}
// ===================== FIN DEL MAPA - Componente RobotMap =====================

// Importamos el componente de mapa de forma dinámica para evitar problemas de SSR
const RouteMap = dynamic(() => import("@/app/components/RouteMap"), { 
  ssr: false,
  loading: () => (
    <div className="bg-gray-200 rounded-lg h-64 md:h-80 flex items-center justify-center">
      <p className="text-gray-500">Cargando mapa...</p>
    </div>
  )
})

export default function Routes() {
  const [selectedRoute, setSelectedRoute] = useState<string | null>(null)
  const [isNavigating, setIsNavigating] = useState(false)
  const [isConnected, setIsConnected] = useState(false)
  const [navigationStatus, setNavigationStatus] = useState<string | null>(null)
  const [currentWaypoint, setCurrentWaypoint] = useState<number | null>(null)
  const [hasAnnouncedArrival, setHasAnnouncedArrival] = useState(false)
  const rosService = useRef<ROSService | null>(null)
  const statusListener = useRef<any>(null)

  // Inicializar el servicio ROS
  useEffect(() => {
    rosService.current = ROSService.getInstance()
    
    // Manejar estado de conexión
    const handleConnectionStatus = (connected: boolean) => {
      setIsConnected(connected)
      
      // Si nos desconectamos durante la navegación, actualizar el estado
      if (!connected && isNavigating) {
        setIsNavigating(false)
        setNavigationStatus("Desconectado")
      }
    }
    
    rosService.current.addConnectionListener(handleConnectionStatus)
    
    // Limpiar al desmontar
    return () => {
      if (rosService.current) {
        rosService.current.removeConnectionListener(handleConnectionStatus)
      }
      if (statusListener.current) {
        statusListener.current.unsubscribe()
      }
    }
  }, [isNavigating])
  
  // Función para iniciar la navegación
  const startNavigation = () => {
    if (!rosService.current || !isConnected) return
    
    const success = rosService.current.startWaypointFollowing()
    if (success) {
      setIsNavigating(true)
      setHasAnnouncedArrival(false)
      setNavigationStatus("Iniciando navegación...")
      
      // Suscribirse al estado de la navegación
      statusListener.current = rosService.current.getNavigationStatus((status, waypoint) => {
        setNavigationStatus(status)
        setCurrentWaypoint(waypoint)
      })
    }
  }
  
  // Función para detener la navegación
  const stopNavigation = () => {
    if (!rosService.current || !isConnected) return
    
    const success = rosService.current.stopWaypointFollowing()
    if (success) {
      setIsNavigating(false)
      setNavigationStatus("Navegación detenida")
      
      // Cancelar suscripción
      if (statusListener.current) {
        statusListener.current.unsubscribe()
        statusListener.current = null
      }
    }
  }

  const savedRoutes = [
    {
      id: "route1",
      name: "Casa - Trabajo",
      distance: "1.5 km",
      duration: "20 min",
      lastUsed: "Hoy",
      description: "Ruta desde casa hasta la oficina pasando por el parque central.",
      waypoints: ["Calle Principal 123", "Parque Central", "Avenida Comercial 45", "Edificio Empresarial"],
    },
    {
      id: "route2",
      name: "Casa - Supermercado",
      distance: "0.8 km",
      duration: "12 min",
      lastUsed: "Ayer",
      description: "Ruta desde casa hasta el supermercado del barrio.",
      waypoints: ["Calle Principal 123", "Plaza del Barrio", "Calle Comercio 78", "Supermercado Mercadona"],
    },
    {
      id: "route3",
      name: "Casa - Centro Médico",
      distance: "2.3 km",
      duration: "30 min",
      lastUsed: "Hace 1 semana",
      description: "Ruta desde casa hasta el centro médico para citas regulares.",
      waypoints: ["Calle Principal 123", "Avenida Central", "Calle Hospital", "Centro Médico Municipal"],
    },
    {
      id: "route4",
      name: "Trabajo - Gimnasio",
      distance: "1.2 km",
      duration: "15 min",
      lastUsed: "Hace 3 días",
      description: "Ruta desde la oficina hasta el gimnasio para las sesiones de tarde.",
      waypoints: ["Edificio Empresarial", "Parque Tecnológico", "Avenida Deportiva", "Gimnasio Fitness"],
    },
  ]

  const recentRoutes = [
    {
      date: "Hoy, 15:30",
      from: "Casa",
      to: "Trabajo",
      duration: "22 min",
    },
    {
      date: "Ayer, 18:45",
      from: "Trabajo",
      to: "Supermercado",
      duration: "15 min",
    },
    {
      date: "Ayer, 19:30",
      from: "Supermercado",
      to: "Casa",
      duration: "14 min",
    },
    {
      date: "15/03/2023, 09:15",
      from: "Casa",
      to: "Centro Médico",
      duration: "28 min",
    },
  ]

  return (
    <div className="bg-background min-h-screen py-16">
      <div className="container-custom">
        <div className="text-center mb-12">
          <h1 className="text-3xl md:text-4xl font-bold mb-4">Rutas del Robot</h1>
          <p className="max-w-3xl mx-auto text-lg">
            Gestiona y visualiza las rutas guardadas y recientes de tu AidGuide.
          </p>
          
          {/* Estado de conexión */}
          <div className={`inline-flex items-center mt-4 px-4 py-2 rounded-full ${isConnected ? 'bg-green-100 text-green-700' : 'bg-red-100 text-red-700'}`}>
            {isConnected ? (
              <>
                <Wifi className="mr-2" size={16} />
                Conectado a ROS
              </>
            ) : (
              <>
                <WifiOff className="mr-2" size={16} />
                Desconectado de ROS
              </>
            )}
          </div>
          
          {/* Estado de navegación */}
          {isNavigating && (
            <div className="inline-flex items-center ml-4 mt-4 px-4 py-2 rounded-full bg-yellow-100 text-yellow-700">
              <Navigation className="mr-2 animate-pulse" size={16} />
              {navigationStatus || "Navegando..."}
              {currentWaypoint !== null && ` (Waypoint ${currentWaypoint})`}
            </div>
          )}
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
          {/* Saved Routes */}
          <div className="lg:col-span-1">
            <div className="bg-white rounded-lg shadow-md p-6">
              <h2 className="text-xl font-bold mb-4 flex items-center">
                <MapPin className="mr-2 text-button" />
                Rutas Guardadas
              </h2>

              <div className="space-y-4">
                {savedRoutes.map((route) => (
                  <div
                    key={route.id}
                    className={`p-4 rounded-lg cursor-pointer transition-colors ${
                      selectedRoute === route.id ? "bg-button text-white" : "bg-gray-100 hover:bg-gray-200"
                    }`}
                    onClick={() => setSelectedRoute(route.id)}
                  >
                    <h3 className="font-bold">{route.name}</h3>
                    <div className="flex justify-between mt-2 text-sm">
                      <span>{route.distance}</span>
                      <span>{route.duration}</span>
                    </div>
                    <div className="text-sm mt-1">
                      <span>Último uso: {route.lastUsed}</span>
                    </div>
                  </div>
                ))}
              </div>

              <button className="btn-primary w-full mt-6 flex items-center justify-center">
                <Navigation className="mr-2" size={18} />
                Nueva Ruta
              </button>
            </div>
          </div>

          {/* Map and Route Details */}
          <div className="lg:col-span-2">
            <div className="bg-white rounded-lg shadow-md p-6 mb-8">
              <h2 className="text-xl font-bold mb-4">Mapa de Ruta</h2>

              {selectedRoute ? (
                <>
                  {/* ===================== INICIO DEL MAPA INTEGRADO ===================== */}
                  <div className="bg-gray-200 rounded-lg h-64 md:h-80 mb-6 overflow-hidden">
                    <RobotMap hasAnnouncedArrival={hasAnnouncedArrival} setHasAnnouncedArrival={setHasAnnouncedArrival} />
                  </div>
                  {/* ===================== FIN DEL MAPA INTEGRADO ===================== */}
                  {selectedRoute && (
                    <RouteMap
                      waypoints={savedRoutes.find((r) => r.id === selectedRoute)?.waypoints || []}
                      routeName={savedRoutes.find((r) => r.id === selectedRoute)?.name || ""}
                    />
                  )}

                  <div>
                    <h3 className="font-bold text-lg mb-2">
                      {savedRoutes.find((r) => r.id === selectedRoute)?.name}
                    </h3>
                    <p className="text-gray-600 mb-4">
                      {savedRoutes.find((r) => r.id === selectedRoute)?.description}
                    </p>

                    <div className="grid grid-cols-2 gap-4 mb-6">
                      <div className="bg-gray-100 p-3 rounded-lg">
                        <div className="text-sm text-gray-500">Distancia</div>
                        <div className="font-bold">
                          {savedRoutes.find((r) => r.id === selectedRoute)?.distance}
                        </div>
                      </div>
                      <div className="bg-gray-100 p-3 rounded-lg">
                        <div className="text-sm text-gray-500">Duración estimada</div>
                        <div className="font-bold">
                          {savedRoutes.find((r) => r.id === selectedRoute)?.duration}
                        </div>
                      </div>
                    </div>

                    <h4 className="font-bold mb-2">Puntos de ruta</h4>
                    <ul className="space-y-2 mb-6">
                      {savedRoutes
                        .find((r) => r.id === selectedRoute)
                        ?.waypoints.map((waypoint, index) => (
                          <li key={index} className="flex items-center">
                            <div className="bg-button text-white w-6 h-6 rounded-full flex items-center justify-center mr-3 flex-shrink-0">
                              {index + 1}
                            </div>
                            <span>{waypoint}</span>
                          </li>
                        ))}
                    </ul>

                    <div className="flex gap-4">
                      {isNavigating ? (
                        <button 
                          onClick={stopNavigation}
                          className="btn-secondary flex-1 flex items-center justify-center"
                          disabled={!isConnected}
                        >
                          <AlertTriangle className="mr-2" size={18} />
                          Detener Navegación
                        </button>
                      ) : (
                        <button 
                          onClick={startNavigation}
                          className="btn-primary flex-1 flex items-center justify-center"
                          disabled={!isConnected}
                        >
                          <Navigation className="mr-2" size={18} />
                          Iniciar Navegación
                        </button>
                      )}
                      <button className="btn-secondary flex-1 flex items-center justify-center">
                        <RotateCw className="mr-2" size={18} />
                        Editar Ruta
                      </button>
                    </div>
                    
                    {/* Mensaje de conexión */}
                    {!isConnected && (
                      <div className="mt-4 p-3 bg-red-100 text-red-700 rounded-lg flex items-center">
                        <WifiOff className="mr-2" size={18} />
                        <span>No se puede iniciar la navegación. Verifica la conexión con ROS.</span>
                      </div>
                    )}
                  </div>
                </>
              ) : (
                <div className="bg-gray-100 rounded-lg h-64 md:h-96 flex items-center justify-center">
                  <div className="text-center p-6">
                    <MapPin className="mx-auto mb-4 text-gray-400" size={48} />
                    <p className="text-gray-500 mb-4">
                      Selecciona una ruta para ver los detalles y el mapa
                    </p>
                    <button className="btn-primary">Crear Nueva Ruta</button>
                  </div>
                </div>
              )}
            </div>

            {/* Recent Routes */}
            <div className="bg-white rounded-lg shadow-md p-6">
              <h2 className="text-xl font-bold mb-4 flex items-center">
                <Clock className="mr-2 text-button" />
                Rutas Recientes
              </h2>

              <div className="overflow-x-auto">
                <table className="w-full">
                  <thead>
                    <tr className="border-b">
                      <th className="text-left py-2 px-4">Fecha</th>
                      <th className="text-left py-2 px-4">Origen</th>
                      <th className="text-left py-2 px-4">Destino</th>
                      <th className="text-left py-2 px-4">Duración</th>
                      <th className="text-left py-2 px-4"></th>
                    </tr>
                  </thead>
                  <tbody>
                    {recentRoutes.map((route, index) => (
                      <tr key={index} className="border-b hover:bg-gray-50">
                        <td className="py-3 px-4">{route.date}</td>
                        <td className="py-3 px-4">{route.from}</td>
                        <td className="py-3 px-4">{route.to}</td>
                        <td className="py-3 px-4">{route.duration}</td>
                        <td className="py-3 px-4">
                          <button className="text-button hover:underline">Ver detalles</button>
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>

              <div className="mt-4 text-center">
                <button className="text-button hover:underline flex items-center mx-auto">
                  <RotateCw className="mr-1" size={16} />
                  Ver historial completo
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
