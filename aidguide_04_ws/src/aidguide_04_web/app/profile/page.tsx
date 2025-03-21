"use client"

import type React from "react"

import { useState } from "react"
import Image from "next/image"
import { User, Settings, MapPin, Bell, Clock, LogOut, Edit2, Save, X } from "lucide-react"

export default function Profile() {
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

  const [activeTab, setActiveTab] = useState("profile")

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
    <div className="bg-background min-h-screen py-16">
      <div className="container-custom">
        <div className="bg-white rounded-lg shadow-md overflow-hidden">
          {/* Profile Header */}
          <div className="bg-button text-white p-6">
            <div className="flex flex-col md:flex-row items-center">
              <div className="w-24 h-24 bg-white rounded-full overflow-hidden mb-4 md:mb-0 md:mr-6">
                <Image
                  src="/placeholder.svg?height=200&width=200"
                  alt="Foto de perfil"
                  width={96}
                  height={96}
                  className="object-cover w-full h-full"
                />
              </div>
              <div>
                <h1 className="text-2xl md:text-3xl font-bold">{userData.name}</h1>
                <p className="text-button-secondary">Usuario desde enero 2023</p>
              </div>
              {!editing ? (
                <button
                  className="ml-auto bg-white text-button py-2 px-4 rounded-md flex items-center"
                  onClick={() => setEditing(true)}
                >
                  <Edit2 size={18} className="mr-2" />
                  Editar Perfil
                </button>
              ) : (
                <div className="ml-auto flex gap-2">
                  <button
                    className="bg-white text-green-600 py-2 px-4 rounded-md flex items-center"
                    onClick={handleSave}
                  >
                    <Save size={18} className="mr-2" />
                    Guardar
                  </button>
                  <button
                    className="bg-white text-red-600 py-2 px-4 rounded-md flex items-center"
                    onClick={handleCancel}
                  >
                    <X size={18} className="mr-2" />
                    Cancelar
                  </button>
                </div>
              )}
            </div>
          </div>

          {/* Tabs */}
          <div className="border-b">
            <div className="flex overflow-x-auto">
              <button
                className={`py-4 px-6 font-medium ${
                  activeTab === "profile" ? "border-b-2 border-button text-button" : "text-gray-500 hover:text-button"
                }`}
                onClick={() => setActiveTab("profile")}
              >
                <User size={18} className="inline mr-2" />
                Perfil
              </button>
              <button
                className={`py-4 px-6 font-medium ${
                  activeTab === "preferences"
                    ? "border-b-2 border-button text-button"
                    : "text-gray-500 hover:text-button"
                }`}
                onClick={() => setActiveTab("preferences")}
              >
                <Settings size={18} className="inline mr-2" />
                Preferencias
              </button>
              <button
                className={`py-4 px-6 font-medium ${
                  activeTab === "activity" ? "border-b-2 border-button text-button" : "text-gray-500 hover:text-button"
                }`}
                onClick={() => setActiveTab("activity")}
              >
                <Clock size={18} className="inline mr-2" />
                Actividad
              </button>
            </div>
          </div>

          {/* Tab Content */}
          <div className="p-6">
            {/* Profile Tab */}
            {activeTab === "profile" && (
              <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
                <div>
                  <h2 className="text-xl font-bold mb-4">Información Personal</h2>

                  <div className="space-y-4">
                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-1">Nombre completo</label>
                      {editing ? (
                        <input
                          type="text"
                          name="name"
                          value={userData.name}
                          onChange={handleChange}
                          className="form-input"
                        />
                      ) : (
                        <p className="text-gray-900">{userData.name}</p>
                      )}
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-1">Correo electrónico</label>
                      {editing ? (
                        <input
                          type="email"
                          name="email"
                          value={userData.email}
                          onChange={handleChange}
                          className="form-input"
                        />
                      ) : (
                        <p className="text-gray-900">{userData.email}</p>
                      )}
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-1">Teléfono</label>
                      {editing ? (
                        <input
                          type="tel"
                          name="phone"
                          value={userData.phone}
                          onChange={handleChange}
                          className="form-input"
                        />
                      ) : (
                        <p className="text-gray-900">{userData.phone}</p>
                      )}
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-1">Dirección</label>
                      {editing ? (
                        <textarea
                          name="address"
                          value={userData.address}
                          onChange={handleChange}
                          className="form-input"
                          rows={2}
                        />
                      ) : (
                        <p className="text-gray-900">{userData.address}</p>
                      )}
                    </div>
                  </div>
                </div>

                <div>
                  <h2 className="text-xl font-bold mb-4">Información de Emergencia</h2>

                  <div className="space-y-4">
                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-1">Contacto de emergencia</label>
                      {editing ? (
                        <input
                          type="text"
                          name="emergencyContact"
                          value={userData.emergencyContact}
                          onChange={handleChange}
                          className="form-input"
                        />
                      ) : (
                        <p className="text-gray-900">{userData.emergencyContact}</p>
                      )}
                    </div>
                  </div>

                  <h2 className="text-xl font-bold mt-8 mb-4">Seguridad</h2>

                  <div className="space-y-4">
                    <button className="btn-secondary">Cambiar contraseña</button>

                    <button className="text-red-600 hover:underline flex items-center">
                      <LogOut size={18} className="mr-2" />
                      Cerrar sesión en todos los dispositivos
                    </button>
                  </div>
                </div>
              </div>
            )}

            {/* Preferences Tab */}
            {activeTab === "preferences" && (
              <div>
                <h2 className="text-xl font-bold mb-4">Preferencias del Robot</h2>

                <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
                  <div>
                    <h3 className="font-bold mb-4">Configuración de Audio</h3>

                    <div className="space-y-6">
                      <div>
                        <label className="block text-sm font-medium text-gray-700 mb-1">
                          Volumen de voz: {userData.preferences.voiceVolume}%
                        </label>
                        <input
                          type="range"
                          name="voiceVolume"
                          min="0"
                          max="100"
                          value={userData.preferences.voiceVolume}
                          onChange={handlePreferenceChange}
                          className="w-full"
                        />
                      </div>

                      <div>
                        <label className="block text-sm font-medium text-gray-700 mb-1">
                          Velocidad de habla: {userData.preferences.speechRate}%
                        </label>
                        <input
                          type="range"
                          name="speechRate"
                          min="0"
                          max="100"
                          value={userData.preferences.speechRate}
                          onChange={handlePreferenceChange}
                          className="w-full"
                        />
                      </div>
                    </div>
                  </div>

                  <div>
                    <h3 className="font-bold mb-4">Configuración de Accesibilidad</h3>

                    <div className="space-y-4">
                      <div className="flex items-center">
                        <input
                          type="checkbox"
                          id="notificationsEnabled"
                          name="notificationsEnabled"
                          checked={userData.preferences.notificationsEnabled}
                          onChange={handlePreferenceChange}
                          className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                        />
                        <label htmlFor="notificationsEnabled" className="ml-2 block text-sm text-gray-900">
                          Activar notificaciones de voz
                        </label>
                      </div>

                      <div className="flex items-center">
                        <input
                          type="checkbox"
                          id="highContrastMode"
                          name="highContrastMode"
                          checked={userData.preferences.highContrastMode}
                          onChange={handlePreferenceChange}
                          className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                        />
                        <label htmlFor="highContrastMode" className="ml-2 block text-sm text-gray-900">
                          Modo de alto contraste
                        </label>
                      </div>

                      <div className="flex items-center">
                        <input
                          type="checkbox"
                          id="largeText"
                          name="largeText"
                          checked={userData.preferences.largeText}
                          onChange={handlePreferenceChange}
                          className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                        />
                        <label htmlFor="largeText" className="ml-2 block text-sm text-gray-900">
                          Texto grande
                        </label>
                      </div>
                    </div>
                  </div>
                </div>

                <div className="mt-8">
                  <h3 className="font-bold mb-4">Configuración de Navegación</h3>

                  <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                    <div className="card">
                      <h4 className="font-medium mb-2">Velocidad de desplazamiento</h4>
                      <select className="form-input">
                        <option>Lenta</option>
                        <option selected>Media</option>
                        <option>Rápida</option>
                      </select>
                    </div>

                    <div className="card">
                      <h4 className="font-medium mb-2">Distancia de detección de obstáculos</h4>
                      <select className="form-input">
                        <option>Corta (1m)</option>
                        <option selected>Media (2m)</option>
                        <option>Larga (3m)</option>
                      </select>
                    </div>

                    <div className="card">
                      <h4 className="font-medium mb-2">Frecuencia de actualización de ruta</h4>
                      <select className="form-input">
                        <option>Baja (cada 10s)</option>
                        <option selected>Media (cada 5s)</option>
                        <option>Alta (cada 2s)</option>
                      </select>
                    </div>

                    <div className="card">
                      <h4 className="font-medium mb-2">Tipo de indicaciones</h4>
                      <select className="form-input">
                        <option>Básicas</option>
                        <option selected>Detalladas</option>
                        <option>Muy detalladas</option>
                      </select>
                    </div>
                  </div>
                </div>

                <div className="mt-8 flex justify-end">
                  <button className="btn-primary">Guardar Preferencias</button>
                </div>
              </div>
            )}

            {/* Activity Tab */}
            {activeTab === "activity" && (
              <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
                <div>
                  <h2 className="text-xl font-bold mb-4">Actividad Reciente</h2>

                  <div className="space-y-4">
                    {recentActivities.map((activity, index) => (
                      <div key={index} className="flex items-start p-3 bg-gray-50 rounded-lg">
                        <div
                          className={`p-2 rounded-full mr-3 flex-shrink-0 ${
                            activity.type === "route"
                              ? "bg-green-100 text-green-600"
                              : activity.type === "settings"
                                ? "bg-blue-100 text-blue-600"
                                : "bg-yellow-100 text-yellow-600"
                          }`}
                        >
                          {activity.icon}
                        </div>
                        <div>
                          <p className="font-medium">{activity.description}</p>
                          <p className="text-sm text-gray-500">{activity.date}</p>
                        </div>
                      </div>
                    ))}
                  </div>

                  <button className="text-button hover:underline mt-4 flex items-center">
                    <Clock className="mr-1" size={16} />
                    Ver historial completo
                  </button>
                </div>

                <div>
                  <h2 className="text-xl font-bold mb-4">Próximas Citas</h2>

                  {upcomingAppointments.length > 0 ? (
                    <div className="space-y-4">
                      {upcomingAppointments.map((appointment, index) => (
                        <div key={index} className="p-4 border rounded-lg">
                          <h3 className="font-bold">{appointment.title}</h3>
                          <div className="mt-2 grid grid-cols-2 gap-2 text-sm">
                            <div>
                              <span className="text-gray-500">Fecha:</span>
                              <span className="ml-1">{appointment.date}</span>
                            </div>
                            <div>
                              <span className="text-gray-500">Hora:</span>
                              <span className="ml-1">{appointment.time}</span>
                            </div>
                            <div className="col-span-2">
                              <span className="text-gray-500">Ubicación:</span>
                              <span className="ml-1">{appointment.location}</span>
                            </div>
                          </div>
                        </div>
                      ))}
                    </div>
                  ) : (
                    <p className="text-gray-500">No tienes citas programadas.</p>
                  )}

                  <div className="mt-6">
                    <h2 className="text-xl font-bold mb-4">Estado del Robot</h2>

                    <div className="grid grid-cols-2 gap-4">
                      <div className="p-4 bg-gray-50 rounded-lg">
                        <h3 className="font-medium text-gray-500">Batería</h3>
                        <div className="flex items-center mt-1">
                          <div className="w-full bg-gray-200 rounded-full h-2.5 mr-2">
                            <div className="bg-green-600 h-2.5 rounded-full" style={{ width: "85%" }}></div>
                          </div>
                          <span className="text-sm font-medium">85%</span>
                        </div>
                      </div>

                      <div className="p-4 bg-gray-50 rounded-lg">
                        <h3 className="font-medium text-gray-500">Conectividad</h3>
                        <p className="text-green-600 font-medium">Conectado (4G)</p>
                      </div>

                      <div className="p-4 bg-gray-50 rounded-lg">
                        <h3 className="font-medium text-gray-500">Versión de Software</h3>
                        <p>v2.3.5</p>
                      </div>

                      <div className="p-4 bg-gray-50 rounded-lg">
                        <h3 className="font-medium text-gray-500">Último Mantenimiento</h3>
                        <p>15/02/2023</p>
                      </div>
                    </div>

                    <button className="btn-secondary mt-4 w-full">Solicitar Mantenimiento</button>
                  </div>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  )
}

