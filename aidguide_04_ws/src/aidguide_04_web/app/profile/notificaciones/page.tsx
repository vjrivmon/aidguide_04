"use client"

import { useState, useEffect } from "react"
import { useRobot } from "@/context/robot-context"
import { CloudRain, Sun, CloudSnow, Cloud, Wind, AlertTriangle, Zap, Eye, Info, Clock, CheckCircle } from "lucide-react"
import { useRouter } from "next/navigation"
import { format } from 'date-fns'
import { es } from 'date-fns/locale'

export default function NotificacionesPage() {
  const { notifications, markNotificationAsRead } = useRobot()
  const router = useRouter()
  const [filteredNotifications, setFilteredNotifications] = useState([...notifications])
  const [filter, setFilter] = useState("all")

  useEffect(() => {
    if (filter === "all") {
      setFilteredNotifications([...notifications])
    } else {
      setFilteredNotifications(notifications.filter(notification => notification.type === filter))
    }
  }, [notifications, filter])

  // Función para obtener el icono según el tipo de clima
  const getWeatherIcon = (type: string) => {
    switch (type?.toLowerCase()) {
      case 'lluvia':
        return <CloudRain size={24} className="text-blue-500" />
      case 'calor':
        return <Sun size={24} className="text-red-500" />
      case 'nieve':
        return <CloudSnow size={24} className="text-white border border-gray-200" />
      case 'niebla':
        return <Cloud size={24} className="text-gray-400" />
      case 'viento':
        return <Wind size={24} className="text-orange-500" />
      case 'tormenta':
        return <Zap size={24} className="text-purple-500" />
      case 'granizo':
        return <AlertTriangle size={24} className="text-cyan-500" />
      default:
        return <Info size={24} className="text-blue-500" />
    }
  }

  // Función para obtener el color de fondo según el tipo de clima
  const getWeatherBgColor = (color: string) => {
    switch (color) {
      case 'blue':
        return 'bg-blue-100'
      case 'red':
        return 'bg-red-100'
      case 'white':
        return 'bg-gray-100'
      case 'gray':
        return 'bg-gray-200'
      case 'orange':
        return 'bg-orange-100'
      case 'purple':
        return 'bg-purple-100'
      case 'cyan':
        return 'bg-cyan-100'
      default:
        return 'bg-blue-50'
    }
  }

  return (
    <div className="container-custom py-8">
      <div className="flex items-center justify-between mb-8">
        <div>
          <h1 className="text-3xl font-bold mb-2">Notificaciones Meteorológicas</h1>
          <p className="text-gray-600">
            Consulta las últimas alertas del tiempo para tu ubicación
          </p>
        </div>
        <button 
          onClick={() => router.push('/profile')}
          className="px-4 py-2 bg-gray-200 rounded-md hover:bg-gray-300 transition-colors"
        >
          Volver al perfil
        </button>
      </div>

      <div className="flex flex-wrap gap-4 mb-6">
        <button
          onClick={() => setFilter("all")}
          className={`px-4 py-2 rounded-md transition-colors ${filter === "all" ? "bg-primary text-white" : "bg-gray-200"}`}
        >
          Todas
        </button>
        <button
          onClick={() => setFilter("weather")}
          className={`px-4 py-2 rounded-md transition-colors ${filter === "weather" ? "bg-primary text-white" : "bg-gray-200"}`}
        >
          Meteorología
        </button>
      </div>

      <div className="bg-white rounded-lg shadow-md p-6">
        {filteredNotifications.length === 0 ? (
          <div className="text-center py-12">
            <Cloud className="mx-auto text-gray-400 mb-4" size={48} />
            <h3 className="text-xl font-medium text-gray-600">No hay notificaciones</h3>
            <p className="text-gray-500 mt-2">Las alertas aparecerán aquí cuando haya cambios en el clima</p>
          </div>
        ) : (
          <div className="space-y-4">
            {filteredNotifications.map(notification => (
              <div 
                key={notification.id} 
                className={`p-4 rounded-lg border ${notification.read ? 'bg-gray-50' : getWeatherBgColor(notification.color || 'blue')} transition-all hover:shadow-md`}
              >
                <div className="flex items-start gap-4">
                  <div className="flex-shrink-0">
                    {notification.type === 'weather' 
                      ? getWeatherIcon(notification.subtype || '') 
                      : <Info size={24} className="text-blue-500" />
                    }
                  </div>
                  <div className="flex-grow">
                    <div className="flex items-center justify-between">
                      <h3 className="font-medium">
                        {notification.subtype 
                          ? `Alerta: ${notification.subtype.charAt(0).toUpperCase() + notification.subtype.slice(1)}` 
                          : 'Notificación'
                        }
                      </h3>
                      <div className="flex items-center text-sm text-gray-500">
                        <Clock size={14} className="mr-1" />
                        {format(new Date(notification.timestamp), 'dd/MM/yyyy HH:mm', {locale: es})}
                      </div>
                    </div>
                    <p className="text-gray-800 mt-1">{notification.message}</p>
                    
                    {!notification.read && (
                      <button 
                        onClick={() => markNotificationAsRead(notification.id)}
                        className="mt-2 flex items-center text-sm text-primary hover:text-primary-dark"
                      >
                        <CheckCircle size={14} className="mr-1" />
                        Marcar como leída
                      </button>
                    )}
                  </div>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      {/* Leyenda de colores */}
      <div className="mt-8 bg-white rounded-lg shadow-md p-6">
        <h3 className="text-xl font-medium mb-4">Leyenda de fenómenos atmosféricos</h3>
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
          <div className="flex items-center gap-3 p-3 bg-blue-100 rounded-md">
            <CloudRain size={24} className="text-blue-500" />
            <span>Lluvia</span>
          </div>
          <div className="flex items-center gap-3 p-3 bg-red-100 rounded-md">
            <Sun size={24} className="text-red-500" />
            <span>Calor</span>
          </div>
          <div className="flex items-center gap-3 p-3 bg-gray-100 rounded-md">
            <CloudSnow size={24} className="text-gray-600" />
            <span>Nieve</span>
          </div>
          <div className="flex items-center gap-3 p-3 bg-gray-200 rounded-md">
            <Cloud size={24} className="text-gray-400" />
            <span>Niebla</span>
          </div>
          <div className="flex items-center gap-3 p-3 bg-orange-100 rounded-md">
            <Wind size={24} className="text-orange-500" />
            <span>Viento</span>
          </div>
          <div className="flex items-center gap-3 p-3 bg-purple-100 rounded-md">
            <Zap size={24} className="text-purple-500" />
            <span>Tormenta</span>
          </div>
          <div className="flex items-center gap-3 p-3 bg-cyan-100 rounded-md">
            <AlertTriangle size={24} className="text-cyan-500" />
            <span>Granizo</span>
          </div>
        </div>
      </div>
    </div>
  )
} 