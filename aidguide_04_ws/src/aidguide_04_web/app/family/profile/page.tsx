"use client"

import { useState } from "react"
import { User, Bell, Settings, History, MapPin, Clock } from "lucide-react"
import { useAuth } from "@/context/auth-context"

export default function FamilyProfile() {
  const { user } = useAuth()
  const [activeTab, setActiveTab] = useState("activity")

  return (
    <div className="container-custom py-14">
      {/* Título y subtítulo */}
      <div className="text-center mb-12">
        <h1 className="text-4xl md:text-4xl font-bold mb-4">
          Perfil Familiar
        </h1>
        <h2 className="text-2xl text-text">
          Gestiona tu cuenta y preferencias
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
              Actividad de María
            </button>
            <button
              onClick={() => setActiveTab("notifications")}
              className={`w-full flex items-center px-4 py-3 rounded-lg transition-colors ${
                activeTab === "notifications"
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
              <h2 className="text-2xl font-bold text-button mb-6">Actividad reciente de María</h2>
              <div className="space-y-4">
                {/* Rutas completadas */}
                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center justify-between mb-2">
                    <div className="flex items-center">
                      <MapPin className="text-button mr-2" size={20} />
                      <span className="font-medium">Ruta completada</span>
                    </div>
                    <span className="text-sm text-gray-500">Hoy, 15:30</span>
                  </div>
                  <p className="text-sm text-gray-600">Desde: Calle Mayor 123</p>
                  <p className="text-sm text-gray-600">Hasta: Plaza Central</p>
                </div>

                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center justify-between mb-2">
                    <div className="flex items-center">
                      <MapPin className="text-button mr-2" size={20} />
                      <span className="font-medium">Ruta completada</span>
                    </div>
                    <span className="text-sm text-gray-500">Ayer, 10:15</span>
                  </div>
                  <p className="text-sm text-gray-600">Desde: Parque Municipal</p>
                  <p className="text-sm text-gray-600">Hasta: Centro Comercial</p>
                </div>
              </div>
            </div>
          )}

          {activeTab === "notifications" && (
            <div>
              <h2 className="text-2xl font-bold text-button mb-6">Notificaciones</h2>
              <div className="space-y-4">
                {/* Notificaciones */}
                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center justify-between mb-2">
                    <div className="flex items-center">
                      <Bell className="text-button mr-2" size={20} />
                      <span className="font-medium">Actualización de estado</span>
                    </div>
                    <span className="text-sm text-gray-500">Hoy, 16:45</span>
                  </div>
                  <p className="text-sm text-gray-600">El robot ha llegado a su destino</p>
                </div>

                <div className="bg-gray-50 p-4 rounded-lg">
                  <div className="flex items-center justify-between mb-2">
                    <div className="flex items-center">
                      <Bell className="text-button mr-2" size={20} />
                      <span className="font-medium">Mantenimiento programado</span>
                    </div>
                    <span className="text-sm text-gray-500">Ayer, 09:30</span>
                  </div>
                  <p className="text-sm text-gray-600">Próxima revisión programada para el 20/03/2024</p>
                </div>
              </div>
            </div>
          )}

          {activeTab === "preferences" && (
            <div>
              <h2 className="text-2xl font-bold text-button mb-6">Preferencias</h2>
              <div className="space-y-6">
                {/* Información personal */}
                <div>
                  <h3 className="text-lg font-medium text-text mb-4">Información Personal</h3>
                  <div className="space-y-4">
                    <div>
                      <label className="block text-sm text-gray-600 mb-1">Nombre</label>
                      <input
                        type="text"
                        value={user?.name || ""}
                        readOnly
                        className="w-full p-2 border border-gray-300 rounded-lg bg-gray-50"
                      />
                    </div>
                    <div>
                      <label className="block text-sm text-gray-600 mb-1">Email</label>
                      <input
                        type="email"
                        value={user?.email || ""}
                        readOnly
                        className="w-full p-2 border border-gray-300 rounded-lg bg-gray-50"
                      />
                    </div>
                  </div>
                </div>

                {/* Preferencias de notificaciones */}
                <div>
                  <h3 className="text-lg font-medium text-text mb-4">Preferencias de Notificaciones</h3>
                  <div className="space-y-4">
                    <div className="flex items-center justify-between">
                      <div>
                        <label className="block text-sm font-medium text-gray-700">Notificaciones de llegada</label>
                        <p className="text-sm text-gray-500">Recibir notificación cuando el robot llegue a su destino</p>
                      </div>
                      <button className="relative inline-flex h-6 w-11 items-center rounded-full bg-button">
                        <span className="sr-only">Activar notificaciones de llegada</span>
                        <span className="inline-block h-4 w-4 transform rounded-full bg-white transition"></span>
                      </button>
                    </div>
                    <div className="flex items-center justify-between">
                      <div>
                        <label className="block text-sm font-medium text-gray-700">Alertas de batería</label>
                        <p className="text-sm text-gray-500">Recibir alerta cuando la batería esté baja</p>
                      </div>
                      <button className="relative inline-flex h-6 w-11 items-center rounded-full bg-button">
                        <span className="sr-only">Activar alertas de batería</span>
                        <span className="inline-block h-4 w-4 transform rounded-full bg-white transition"></span>
                      </button>
                    </div>
                  </div>
                </div>

                {/* Preferencias de visualización */}
                <div>
                  <h3 className="text-lg font-medium text-text mb-4">Preferencias de Visualización</h3>
                  <div className="space-y-4">
                    <div>
                      <label className="block text-sm text-gray-600 mb-1">Idioma</label>
                      <select className="w-full p-2 border border-gray-300 rounded-lg">
                        <option value="es">Español</option>
                        <option value="en">English</option>
                      </select>
                    </div>
                    <div>
                      <label className="block text-sm text-gray-600 mb-1">Zona horaria</label>
                      <select className="w-full p-2 border border-gray-300 rounded-lg">
                        <option value="UTC+1">Madrid (UTC+1)</option>
                        <option value="UTC+0">Londres (UTC+0)</option>
                      </select>
                    </div>
                  </div>
                </div>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  )
} 