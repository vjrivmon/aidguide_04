"use client"

import type React from "react"

import { useState } from "react"
import Image from "next/image"
import { User, Settings, MapPin, Bell, Clock, LogOut, Edit2, Save, X, History, Camera, TrafficCone, Signpost, Bus, Users, Footprints, Wrench } from "lucide-react"
import { useAuth } from "@/context/auth-context"

export default function Profile() {
  const { user } = useAuth()
  const [editing, setEditing] = useState(false)
  const [userData, setUserData] = useState({
    name: "María García",
    email: "maria.garcia@example.com",
    phone: "+34 612 345 678",
    address: "Calle Principal 123, Valencia",
    emergencyContact: "Juan García - +34 698 765 432",
    preferences: {
      voiceVolume: 80,
      speechRate: 60,
      notificationsEnabled: true,
      highContrastMode: false,
      largeText: true,
    },
  })

  const [activeTab, setActiveTab] = useState("activity")

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target
    setUserData((prev) => ({
      ...prev,
      [name]: value,
    }))
  }

  const handlePreferenceChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value, type, checked } = e.target
    setUserData((prev) => ({
      ...prev,
      preferences: {
        ...prev.preferences,
        [name]: type === "checkbox" ? checked : Number(value),
      },
    }))
  }

  const handleSave = () => {
    // Aquí iría la lógica para guardar los cambios en el servidor
    setEditing(false)
  }

  const handleCancel = () => {
    // Revertir cambios
    setEditing(false)
  }

  const recentActivities = [
    {
      type: "route",
      description: "Ruta completada: Casa - Trabajo",
      date: "Hoy, 09:15",
      icon: <MapPin size={16} />,
    },
    {
      type: "settings",
      description: "Configuración actualizada: Volumen de voz",
      date: "Ayer, 18:30",
      icon: <Settings size={16} />,
    },
    {
      type: "notification",
      description: "Alerta: Batería baja (20%)",
      date: "Ayer, 16:45",
      icon: <Bell size={16} />,
    },
    {
      type: "route",
      description: "Ruta completada: Trabajo - Supermercado - Casa",
      date: "15/03/2023, 19:20",
      icon: <MapPin size={16} />,
    },
  ]

  const upcomingAppointments = [
    {
      title: "Mantenimiento preventivo",
      date: "25/03/2023",
      time: "10:00 - 11:00",
      location: "Centro de servicio AidGuide",
    },
    {
      title: "Actualización de software",
      date: "02/04/2023",
      time: "Automática",
      location: "Remoto",
    },
  ]

  return (
    <div className="container-custom py-14">
      {/* Título y subtítulo */}
      <div className="text-center mb-12">
        <h1 className="text-4xl md:text-4xl font-bold mb-4">
          ¡Hola, {user?.name}!
        </h1>
        <h2 className="text-2xl text-text">
          Tu perfil
        </h2>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
        {/* Panel izquierdo - Navegación */}
        <div className="bg-white rounded-lg shadow-md p-6">
          <h2 className="text-2xl font-bold text-button mb-6">Menú</h2>
          <nav className="space-y-2">
            <button
              onClick={() => setActiveTab("activity")}
              className={`w-full flex items-center px-4 py-3 rounded-lg transition-colors ${
                activeTab === "activity"
                  ? "bg-button text-white"
                  : "text-text hover:bg-gray-50"
              }`}
            >
              <History size={20} className="mr-3" />
              Actividad
            </button>
            <button
              onClick={() => setActiveTab("images")}
              className={`w-full flex items-center px-4 py-3 rounded-lg transition-colors ${
                activeTab === "images"
                  ? "bg-button text-white"
                  : "text-text hover:bg-gray-50"
              }`}
            >
              
              <Bell size={20} className="mr-3" />
              Notificaciones
            </button>
            <button
              onClick={() => setActiveTab("preferences")}
              className={`w-full flex items-center px-4 py-3 rounded-lg transition-colors ${
                activeTab === "preferences"
                  ? "bg-button text-white"
                  : "text-text hover:bg-gray-50"
              }`}
            >
              <Settings size={20} className="mr-3" />
              Preferencias
            </button>
          </nav>
        </div>

        {/* Panel central - Contenido */}
        <div className="md:col-span-2 bg-white rounded-lg shadow-md p-6">
          {activeTab === "activity" && (
            <div>
              <h2 className="text-2xl font-bold text-button mb-6">Actividad reciente</h2>
              <div className="space-y-4">
                <div className="flex items-center justify-between p-4 bg-gray-50 rounded-lg">
                  <div className="flex items-center">
                    <History size={20} className="text-button mr-3" />
                    <div>
                      <p className="font-medium">Ruta completada</p>
                      <p className="text-sm text-gray-500">Desde: Casa - Hasta: Supermercado</p>
                    </div>
                  </div>
                  <span className="text-sm text-gray-500">Hace 2 horas</span>
                </div>
                <div className="flex items-center justify-between p-4 bg-gray-50 rounded-lg">
                  <div className="flex items-center">
                    <History size={20} className="text-button mr-3" />
                    <div>
                      <p className="font-medium">Ruta completada</p>
                      <p className="text-sm text-gray-500">Desde: Supermercado - Hasta: Casa</p>
                    </div>
                  </div>
                  <span className="text-sm text-gray-500">Hace 3 horas</span>
                </div>
              </div>
            </div>
          )}

          {activeTab === "images" && (
            <div>
              <h2 className="text-2xl font-bold text-button mb-6">Imágenes detectadas</h2>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                {/* Señales de tráfico */}
                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center mb-3">
                    <TrafficCone size={20} className="text-button mr-2" />
                    <h3 className="font-medium">Señales de tráfico</h3>
                  </div>
                  <div className="relative h-48 bg-gray-200 rounded-lg overflow-hidden">
                    <Image
                      src="/placeholder.svg?height=200&width=200"
                      alt="Señal de tráfico"
                      fill
                      className="object-cover"
                    />
                  </div>
                  <p className="text-sm text-gray-500 mt-2">Detectado: 15/03/2024, 10:30</p>
                </div>

                {/* Personas */}
                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center mb-3">
                    <Users size={20} className="text-button mr-2" />
                    <h3 className="font-medium">Personas</h3>
                  </div>
                  <div className="relative h-48 bg-gray-200 rounded-lg overflow-hidden">
                    <Image
                      src="/placeholder.svg?height=200&width=200"
                      alt="Personas"
                      fill
                      className="object-cover"
                    />
                  </div>
                  <p className="text-sm text-gray-500 mt-2">Detectado: 15/03/2024, 10:35</p>
                </div>

                {/* Parada de autobús */}
                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center mb-3">
                    <Bus size={20} className="text-button mr-2" />
                    <h3 className="font-medium">Parada de autobús</h3>
                  </div>
                  <div className="relative h-48 bg-gray-200 rounded-lg overflow-hidden">
                    <Image
                      src="/placeholder.svg?height=200&width=200"
                      alt="Parada de autobús"
                      fill
                      className="object-cover"
                    />
                  </div>
                  <p className="text-sm text-gray-500 mt-2">Detectado: 15/03/2024, 10:40</p>
                </div>

                {/* Paso de peatones */}
                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center mb-3">
                    <Footprints size={20} className="text-button mr-2" />
                    <h3 className="font-medium">Paso de peatones</h3>
                  </div>
                  <div className="relative h-48 bg-gray-200 rounded-lg overflow-hidden">
                    <Image
                      src="/placeholder.svg?height=200&width=200"
                      alt="Paso de peatones"
                      fill
                      className="object-cover"
                    />
                  </div>
                  <p className="text-sm text-gray-500 mt-2">Detectado: 15/03/2024, 10:45</p>
                </div>

                {/* Obras */}
                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center mb-3">
                    <Wrench size={20} className="text-button mr-2" />
                    <h3 className="font-medium">Obras</h3>
                  </div>
                  <div className="relative h-48 bg-gray-200 rounded-lg overflow-hidden">
                    <Image
                      src="/placeholder.svg?height=200&width=200"
                      alt="Obras"
                      fill
                      className="object-cover"
                    />
                  </div>
                  <p className="text-sm text-gray-500 mt-2">Detectado: 15/03/2024, 10:50</p>
                </div>

                {/* Calle cortada */}
                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center mb-3">
                    <Signpost size={20} className="text-button mr-2" />
                    <h3 className="font-medium">Calle cortada</h3>
                  </div>
                  <div className="relative h-48 bg-gray-200 rounded-lg overflow-hidden">
                    <Image
                      src="/placeholder.svg?height=200&width=200"
                      alt="Calle cortada"
                      fill
                      className="object-cover"
                    />
                  </div>
                  <p className="text-sm text-gray-500 mt-2">Detectado: 15/03/2024, 10:55</p>
                </div>
              </div>
            </div>
          )}

          {activeTab === "notifications" && (
            <div>
              <h2 className="text-2xl font-bold text-button mb-6">Notificaciones</h2>
              <div className="space-y-4">
                <div className="flex items-center justify-between p-4 bg-gray-50 rounded-lg">
                  <div className="flex items-center">
                    <Bell size={20} className="text-button mr-3" />
                    <div>
                      <p className="font-medium">Nueva actualización</p>
                      <p className="text-sm text-gray-500">El robot ha sido actualizado con nuevas funcionalidades</p>
                    </div>
                  </div>
                  <span className="text-sm text-gray-500">Hace 1 día</span>
                </div>
                <div className="flex items-center justify-between p-4 bg-gray-50 rounded-lg">
                  <div className="flex items-center">
                    <Bell size={20} className="text-button mr-3" />
                    <div>
                      <p className="font-medium">Mantenimiento programado</p>
                      <p className="text-sm text-gray-500">El robot necesita una revisión programada</p>
                    </div>
                  </div>
                  <span className="text-sm text-gray-500">Hace 2 días</span>
                </div>
              </div>
            </div>
          )}

          {activeTab === "preferences" && (
            <div>
              <h2 className="text-2xl font-bold text-button mb-6">Preferencias</h2>
              <div className="space-y-6">
                <div>
                  <label className="block text-text mb-2">Idioma preferido</label>
                  <select className="w-full p-3 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-button">
                    <option value="es">Español</option>
                    <option value="en">English</option>
                  </select>
                </div>
                <div>
                  <label className="block text-text mb-2">Velocidad de navegación</label>
                  <select className="w-full p-3 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-button">
                    <option value="slow">Lenta</option>
                    <option value="medium">Media</option>
                    <option value="fast">Rápida</option>
                  </select>
                </div>
                <div>
                  <label className="block text-text mb-2">Volumen de voz</label>
                  <input
                    type="range"
                    min="0"
                    max="100"
                    className="w-full"
                  />
                </div>
                <button className="w-full bg-button text-white py-3 rounded-lg hover:opacity-90 transition-colors">
                  Guardar cambios
                </button>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}

