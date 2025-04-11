"use client"

import { useState, useEffect, useRef } from "react"
import { MapPin, Navigation, Clock, RotateCw, Wifi, WifiOff, AlertTriangle } from "lucide-react"
import dynamic from "next/dynamic"
import ROSService from "@/services/ros-service"

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
        setNavigationStatus("Desconectado");
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
                  <div className="bg-gray-200 rounded-lg h-64 md:h-80 mb-6 overflow-hidden">
                    {selectedRoute && (
                      <RouteMap 
                        waypoints={savedRoutes.find((r) => r.id === selectedRoute)?.waypoints || []}
                        routeName={savedRoutes.find((r) => r.id === selectedRoute)?.name || ""}
                      />
                    )}
                  </div>

                  <div>
                    <h3 className="font-bold text-lg mb-2">{savedRoutes.find((r) => r.id === selectedRoute)?.name}</h3>
                    <p className="text-gray-600 mb-4">{savedRoutes.find((r) => r.id === selectedRoute)?.description}</p>

                    <div className="grid grid-cols-2 gap-4 mb-6">
                      <div className="bg-gray-100 p-3 rounded-lg">
                        <div className="text-sm text-gray-500">Distancia</div>
                        <div className="font-bold">{savedRoutes.find((r) => r.id === selectedRoute)?.distance}</div>
                      </div>
                      <div className="bg-gray-100 p-3 rounded-lg">
                        <div className="text-sm text-gray-500">Duración estimada</div>
                        <div className="font-bold">{savedRoutes.find((r) => r.id === selectedRoute)?.duration}</div>
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
                    <p className="text-gray-500 mb-4">Selecciona una ruta para ver los detalles y el mapa</p>
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

