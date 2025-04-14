"use client"

import { useState } from "react"
import Link from "next/link"
import { Save, Bell, Shield, Database, Globe, Users, Mail, Smartphone } from "lucide-react"

export default function AdminSettings() {
  const [activeTab, setActiveTab] = useState("general")

  return (
    <div className="bg-background min-h-screen">
      {/* Admin Header */}
      <div className="bg-button text-white p-4">
        <div className="container-custom">
          <div className="flex justify-between items-center">
            <h1 className="text-xl font-bold">Panel de Administración</h1>
            <div className="flex items-center gap-4">
            </div>
          </div>
        </div>
      </div>

      {/* Admin Navigation */}
      <div className="bg-white border-b">
        <div className="container-custom">
          <div className="flex overflow-x-auto">
            <Link href="/admin/dashboard" className="py-4 px-6 text-gray-500 hover:text-button font-medium">
              Dashboard
            </Link>
            <Link href="/admin/users" className="py-4 px-6 text-gray-500 hover:text-button font-medium">
              Usuarios
            </Link>
            <Link href="/admin/robots" className="py-4 px-6 text-gray-500 hover:text-button font-medium">
              Robots
            </Link>
            <Link href="/admin/routes" className="py-4 px-6 text-gray-500 hover:text-button font-medium">
              Rutas
            </Link>
            <Link href="/admin/alerts" className="py-4 px-6 text-gray-500 hover:text-button font-medium">
              Alertas
            </Link>
            <Link href="/admin/settings" className="py-4 px-6 border-b-2 border-button text-button font-medium">
              Configuración
            </Link>
          </div>
        </div>
      </div>

      <div className="container-custom py-8">
        <div className="bg-white rounded-lg shadow-md overflow-hidden">
          <div className="p-4 border-b">
            <h2 className="text-lg font-bold">Configuración del Sistema</h2>
          </div>

          <div className="flex flex-col md:flex-row">
            {/* Sidebar */}
            <div className="w-full md:w-64 border-b md:border-b-0 md:border-r">
              <nav className="p-4">
                <ul className="space-y-1">
                  <li>
                    <button
                      className={`w-full text-left px-4 py-2 rounded-md flex items-center ${
                        activeTab === "general" ? "bg-button text-white" : "hover:bg-gray-100"
                      }`}
                      onClick={() => setActiveTab("general")}
                    >
                      <Globe className="mr-2" size={18} />
                      General
                    </button>
                  </li>
                  <li>
                    <button
                      className={`w-full text-left px-4 py-2 rounded-md flex items-center ${
                        activeTab === "notifications" ? "bg-button text-white" : "hover:bg-gray-100"
                      }`}
                      onClick={() => setActiveTab("notifications")}
                    >
                      <Bell className="mr-2" size={18} />
                      Notificaciones
                    </button>
                  </li>
                  <li>
                    <button
                      className={`w-full text-left px-4 py-2 rounded-md flex items-center ${
                        activeTab === "security" ? "bg-button text-white" : "hover:bg-gray-100"
                      }`}
                      onClick={() => setActiveTab("security")}
                    >
                      <Shield className="mr-2" size={18} />
                      Seguridad
                    </button>
                  </li>
                  <li>
                    <button
                      className={`w-full text-left px-4 py-2 rounded-md flex items-center ${
                        activeTab === "database" ? "bg-button text-white" : "hover:bg-gray-100"
                      }`}
                      onClick={() => setActiveTab("database")}
                    >
                      <Database className="mr-2" size={18} />
                      Base de Datos
                    </button>
                  </li>
                  <li>
                    <button
                      className={`w-full text-left px-4 py-2 rounded-md flex items-center ${
                        activeTab === "users" ? "bg-button text-white" : "hover:bg-gray-100"
                      }`}
                      onClick={() => setActiveTab("users")}
                    >
                      <Users className="mr-2" size={18} />
                      Usuarios
                    </button>
                  </li>
                  <li>
                    <button
                      className={`w-full text-left px-4 py-2 rounded-md flex items-center ${
                        activeTab === "communications" ? "bg-button text-white" : "hover:bg-gray-100"
                      }`}
                      onClick={() => setActiveTab("communications")}
                    >
                      <Mail className="mr-2" size={18} />
                      Comunicaciones
                    </button>
                  </li>
                  <li>
                    <button
                      className={`w-full text-left px-4 py-2 rounded-md flex items-center ${
                        activeTab === "mobile" ? "bg-button text-white" : "hover:bg-gray-100"
                      }`}
                      onClick={() => setActiveTab("mobile")}
                    >
                      <Smartphone className="mr-2" size={18} />
                      Aplicación Móvil
                    </button>
                  </li>
                </ul>
              </nav>
            </div>

            {/* Content */}
            <div className="flex-1 p-6">
              {activeTab === "general" && (
                <div>
                  <h3 className="text-xl font-bold mb-4">Configuración General</h3>

                  <div className="space-y-6">
                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-1">Nombre de la Plataforma</label>
                      <input type="text" className="form-input" defaultValue="AidGuide" />
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-1">Descripción</label>
                      <textarea
                        className="form-input"
                        rows={3}
                        defaultValue="Plataforma para la gestión y uso del robot guía destinado a personas invidentes"
                      />
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-1">Zona Horaria</label>
                      <select className="form-input">
                        <option>Europe/Madrid (GMT+1)</option>
                        <option>Europe/London (GMT+0)</option>
                        <option>America/New_York (GMT-5)</option>
                        <option>Asia/Tokyo (GMT+9)</option>
                      </select>
                    </div>

                    <div>
                      <label className="block text-sm font-medium text-gray-700 mb-1">Idioma Predeterminado</label>
                      <select className="form-input">
                        <option>Español</option>
                        <option>English</option>
                        <option>Français</option>
                        <option>Deutsch</option>
                      </select>
                    </div>

                    <div className="flex items-center">
                      <input
                        type="checkbox"
                        id="maintenance-mode"
                        className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                        defaultChecked={false}
                      />
                      <label htmlFor="maintenance-mode" className="ml-2 block text-sm text-gray-900">
                        Activar modo de mantenimiento
                      </label>
                    </div>
                  </div>

                  <div className="mt-6 flex justify-end">
                    <button className="btn-primary flex items-center">
                      <Save size={18} className="mr-2" />
                      Guardar Cambios
                    </button>
                  </div>
                </div>
              )}

              {activeTab === "notifications" && (
                <div>
                  <h3 className="text-xl font-bold mb-4">Configuración de Notificaciones</h3>

                  <div className="space-y-6">
                    <div>
                      <h4 className="font-medium mb-2">Notificaciones por Email</h4>
                      <div className="space-y-2">
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="email-alerts"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="email-alerts" className="ml-2 block text-sm text-gray-900">
                            Enviar alertas críticas por email
                          </label>
                        </div>
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="email-reports"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="email-reports" className="ml-2 block text-sm text-gray-900">
                            Enviar informes diarios por email
                          </label>
                        </div>
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="email-maintenance"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="email-maintenance" className="ml-2 block text-sm text-gray-900">
                            Notificar mantenimientos programados
                          </label>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium mb-2">Notificaciones Push</h4>
                      <div className="space-y-2">
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="push-alerts"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="push-alerts" className="ml-2 block text-sm text-gray-900">
                            Enviar alertas críticas como notificaciones push
                          </label>
                        </div>
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="push-battery"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="push-battery" className="ml-2 block text-sm text-gray-900">
                            Notificar cuando la batería esté por debajo del 20%
                          </label>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium mb-2">Umbrales de Alerta</h4>
                      <div className="space-y-4">
                        <div>
                          <label className="block text-sm text-gray-700 mb-1">Umbral de batería baja (%)</label>
                          <input type="number" className="form-input w-24" defaultValue={20} min={5} max={50} />
                        </div>
                        <div>
                          <label className="block text-sm text-gray-700 mb-1">Umbral de señal débil (dBm)</label>
                          <input type="number" className="form-input w-24" defaultValue={-80} min={-100} max={-50} />
                        </div>
                      </div>
                    </div>
                  </div>

                  <div className="mt-6 flex justify-end">
                    <button className="btn-primary flex items-center">
                      <Save size={18} className="mr-2" />
                      Guardar Cambios
                    </button>
                  </div>
                </div>
              )}

              {activeTab === "security" && (
                <div>
                  <h3 className="text-xl font-bold mb-4">Configuración de Seguridad</h3>

                  <div className="space-y-6">
                    <div>
                      <h4 className="font-medium mb-2">Política de Contraseñas</h4>
                      <div className="space-y-2">
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="password-complexity"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="password-complexity" className="ml-2 block text-sm text-gray-900">
                            Requerir contraseñas complejas
                          </label>
                        </div>
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="password-expiry"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="password-expiry" className="ml-2 block text-sm text-gray-900">
                            Expirar contraseñas cada 90 días
                          </label>
                        </div>
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="two-factor"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="two-factor" className="ml-2 block text-sm text-gray-900">
                            Habilitar autenticación de dos factores
                          </label>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium mb-2">Sesiones</h4>
                      <div className="space-y-4">
                        <div>
                          <label className="block text-sm text-gray-700 mb-1">
                            Tiempo de expiración de sesión (minutos)
                          </label>
                          <input type="number" className="form-input w-24" defaultValue={30} min={5} max={120} />
                        </div>
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="session-device"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="session-device" className="ml-2 block text-sm text-gray-900">
                            Limitar a una sesión por dispositivo
                          </label>
                        </div>
                      </div>
                    </div>

                    <div>
                      <h4 className="font-medium mb-2">Registro de Actividad</h4>
                      <div className="space-y-2">
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="log-login"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="log-login" className="ml-2 block text-sm text-gray-900">
                            Registrar intentos de inicio de sesión
                          </label>
                        </div>
                        <div className="flex items-center">
                          <input
                            type="checkbox"
                            id="log-actions"
                            className="h-4 w-4 rounded border-gray-300 text-button focus:ring-button"
                            defaultChecked={true}
                          />
                          <label htmlFor="log-actions" className="ml-2 block text-sm text-gray-900">
                            Registrar acciones administrativas
                          </label>
                        </div>
                      </div>
                    </div>
                  </div>

                  <div className="mt-6 flex justify-end">
                    <button className="btn-primary flex items-center">
                      <Save size={18} className="mr-2" />
                      Guardar Cambios
                    </button>
                  </div>
                </div>
              )}

              {/* Otros tabs... */}
              {activeTab !== "general" && activeTab !== "notifications" && activeTab !== "security" && (
                <div className="flex items-center justify-center h-64">
                  <p className="text-gray-500">Contenido en desarrollo</p>
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

