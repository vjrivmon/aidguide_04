"use client"

import { useState } from "react"
import Link from "next/link"
import {
  Users,
  BotIcon as Robot,
  AlertTriangle,
  Activity,
  Map,
  ChevronDown,
  ChevronUp,
  Search,
  Filter,
  Download,
  MoreHorizontal,
} from "lucide-react"

export default function AdminDashboard() {
  const [expandedUser, setExpandedUser] = useState<string | null>(null)

  const toggleUserExpand = (userId: string) => {
    if (expandedUser === userId) {
      setExpandedUser(null)
    } else {
      setExpandedUser(userId)
    }
  }

  const users = [
    {
      id: "user1",
      name: "María García",
      email: "maria.garcia@example.com",
      status: "active",
      lastActive: "Hace 2 horas",
      robotStatus: "online",
      batteryLevel: 85,
      location: "Valencia, España",
      registrationDate: "15/01/2023",
    },
    {
      id: "user2",
      name: "Carlos Rodríguez",
      email: "carlos.rodriguez@example.com",
      status: "active",
      lastActive: "Hace 1 día",
      robotStatus: "offline",
      batteryLevel: 20,
      location: "Barcelona, España",
      registrationDate: "03/02/2023",
    },
    {
      id: "user3",
      name: "Ana Martínez",
      email: "ana.martinez@example.com",
      status: "inactive",
      lastActive: "Hace 15 días",
      robotStatus: "maintenance",
      batteryLevel: 0,
      location: "Madrid, España",
      registrationDate: "22/11/2022",
    },
    {
      id: "user4",
      name: "Javier López",
      email: "javier.lopez@example.com",
      status: "active",
      lastActive: "Hace 5 horas",
      robotStatus: "online",
      batteryLevel: 72,
      location: "Sevilla, España",
      registrationDate: "10/03/2023",
    },
  ]

  const alerts = [
    {
      id: "alert1",
      user: "María García",
      type: "battery",
      message: "Batería baja (15%)",
      time: "Hace 30 minutos",
      severity: "medium",
    },
    {
      id: "alert2",
      user: "Carlos Rodríguez",
      type: "connection",
      message: "Pérdida de conexión",
      time: "Hace 2 horas",
      severity: "high",
    },
    {
      id: "alert3",
      user: "Ana Martínez",
      type: "maintenance",
      message: "Mantenimiento programado pendiente",
      time: "Hace 3 días",
      severity: "low",
    },
  ]

  const stats = [
    {
      title: "Usuarios Activos",
      value: "32",
      change: "+5%",
      icon: <Users className="text-blue-500" />,
    },
    {
      title: "Robots Conectados",
      value: "28",
      change: "+2%",
      icon: <Robot className="text-green-500" />,
    },
    {
      title: "Alertas Activas",
      value: "7",
      change: "-12%",
      icon: <AlertTriangle className="text-yellow-500" />,
    },
    {
      title: "Rutas Completadas",
      value: "156",
      change: "+23%",
      icon: <Map className="text-purple-500" />,
    },
  ]

  return (
    <div className="bg-background min-h-screen">
      {/* Admin Header */}
      <div className="bg-button text-white p-4">
        <div className="container-custom">
          <div className="flex justify-between items-center">
            <h1 className="text-xl font-bold">Panel de Administración</h1>
            <div className="flex items-center gap-4">
              <span>Admin</span>
              <div className="w-8 h-8 bg-white rounded-full"></div>
            </div>
          </div>
        </div>
      </div>

      {/* Admin Navigation */}
      <div className="bg-white border-b">
        <div className="container-custom">
          <div className="flex overflow-x-auto">
            <Link href="/admin/dashboard" className="py-4 px-6 border-b-2 border-button text-button font-medium">
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
            <Link href="/admin/settings" className="py-4 px-6 text-gray-500 hover:text-button font-medium">
              Configuración
            </Link>
          </div>
        </div>
      </div>

      <div className="container-custom py-8">
        {/* Stats Cards */}
        <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-6 mb-8">
          {stats.map((stat, index) => (
            <div key={index} className="bg-white p-6 rounded-lg shadow-md">
              <div className="flex justify-between items-start">
                <div>
                  <p className="text-gray-500 text-sm">{stat.title}</p>
                  <h3 className="text-2xl font-bold mt-1">{stat.value}</h3>
                  <p className={`text-sm mt-1 ${stat.change.startsWith("+") ? "text-green-500" : "text-red-500"}`}>
                    {stat.change} desde el mes pasado
                  </p>
                </div>
                <div className="p-3 bg-gray-100 rounded-full">{stat.icon}</div>
              </div>
            </div>
          ))}
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
          {/* Users List */}
          <div className="lg:col-span-2">
            <div className="bg-white rounded-lg shadow-md overflow-hidden">
              <div className="p-4 border-b flex justify-between items-center">
                <h2 className="text-lg font-bold">Usuarios Recientes</h2>
                <div className="flex gap-2">
                  <button className="p-2 rounded-md hover:bg-gray-100">
                    <Search size={18} />
                  </button>
                  <button className="p-2 rounded-md hover:bg-gray-100">
                    <Filter size={18} />
                  </button>
                  <button className="p-2 rounded-md hover:bg-gray-100">
                    <Download size={18} />
                  </button>
                </div>
              </div>

              <div className="overflow-x-auto">
                <table className="w-full">
                  <thead>
                    <tr className="bg-gray-50">
                      <th className="text-left py-3 px-4 font-medium">Usuario</th>
                      <th className="text-left py-3 px-4 font-medium">Estado</th>
                      <th className="text-left py-3 px-4 font-medium">Robot</th>
                      <th className="text-left py-3 px-4 font-medium">Última Actividad</th>
                      <th className="text-left py-3 px-4 font-medium"></th>
                    </tr>
                  </thead>
                  <tbody>
                    {users.map((user) => (
                      <>
                        <tr key={user.id} className="border-b hover:bg-gray-50">
                          <td className="py-3 px-4">
                            <div>
                              <p className="font-medium">{user.name}</p>
                              <p className="text-sm text-gray-500">{user.email}</p>
                            </div>
                          </td>
                          <td className="py-3 px-4">
                            <span
                              className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
                                user.status === "active" ? "bg-green-100 text-green-800" : "bg-gray-100 text-gray-800"
                              }`}
                            >
                              {user.status === "active" ? "Activo" : "Inactivo"}
                            </span>
                          </td>
                          <td className="py-3 px-4">
                            <span
                              className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
                                user.robotStatus === "online"
                                  ? "bg-green-100 text-green-800"
                                  : user.robotStatus === "offline"
                                    ? "bg-red-100 text-red-800"
                                    : "bg-yellow-100 text-yellow-800"
                              }`}
                            >
                              {user.robotStatus === "online"
                                ? "En línea"
                                : user.robotStatus === "offline"
                                  ? "Desconectado"
                                  : "Mantenimiento"}
                            </span>
                          </td>
                          <td className="py-3 px-4 text-sm">{user.lastActive}</td>
                          <td className="py-3 px-4">
                            <div className="flex items-center gap-2">
                              <button
                                className="text-gray-500 hover:text-button"
                                onClick={() => toggleUserExpand(user.id)}
                                aria-label={expandedUser === user.id ? "Colapsar detalles" : "Expandir detalles"}
                              >
                                {expandedUser === user.id ? <ChevronUp size={18} /> : <ChevronDown size={18} />}
                              </button>
                              <button className="text-gray-500 hover:text-button">
                                <MoreHorizontal size={18} />
                              </button>
                            </div>
                          </td>
                        </tr>
                        {expandedUser === user.id && (
                          <tr className="bg-gray-50">
                            <td colSpan={5} className="py-4 px-6">
                              <div className="grid grid-cols-1 sm:grid-cols-3 gap-4">
                                <div>
                                  <p className="text-sm text-gray-500">Ubicación</p>
                                  <p className="font-medium">{user.location}</p>
                                </div>
                                <div>
                                  <p className="text-sm text-gray-500">Nivel de Batería</p>
                                  <div className="flex items-center mt-1">
                                    <div className="w-full max-w-[100px] bg-gray-200 rounded-full h-2.5 mr-2">
                                      <div
                                        className={`h-2.5 rounded-full ${
                                          user.batteryLevel > 60
                                            ? "bg-green-600"
                                            : user.batteryLevel > 20
                                              ? "bg-yellow-500"
                                              : "bg-red-600"
                                        }`}
                                        style={{ width: `${user.batteryLevel}%` }}
                                      ></div>
                                    </div>
                                    <span className="text-sm font-medium">{user.batteryLevel}%</span>
                                  </div>
                                </div>
                                <div>
                                  <p className="text-sm text-gray-500">Fecha de Registro</p>
                                  <p className="font-medium">{user.registrationDate}</p>
                                </div>
                              </div>
                              <div className="mt-4 flex gap-2">
                                <button className="text-sm text-button hover:underline">Ver perfil completo</button>
                                <button className="text-sm text-button hover:underline">Ver historial de rutas</button>
                                <button className="text-sm text-button hover:underline">Enviar mensaje</button>
                              </div>
                            </td>
                          </tr>
                        )}
                      </>
                    ))}
                  </tbody>
                </table>
              </div>

              <div className="p-4 border-t flex justify-between items-center">
                <p className="text-sm text-gray-500">Mostrando 4 de 32 usuarios</p>
                <div className="flex gap-2">
                  <button className="px-3 py-1 border rounded-md text-sm">Anterior</button>
                  <button className="px-3 py-1 bg-button text-white rounded-md text-sm">1</button>
                  <button className="px-3 py-1 border rounded-md text-sm">2</button>
                  <button className="px-3 py-1 border rounded-md text-sm">3</button>
                  <button className="px-3 py-1 border rounded-md text-sm">Siguiente</button>
                </div>
              </div>
            </div>
          </div>

          {/* Alerts Panel */}
          <div className="lg:col-span-1">
            <div className="bg-white rounded-lg shadow-md overflow-hidden">
              <div className="p-4 border-b">
                <h2 className="text-lg font-bold">Alertas Recientes</h2>
              </div>

              <div className="p-4">
                {alerts.length > 0 ? (
                  <div className="space-y-4">
                    {alerts.map((alert) => (
                      <div
                        key={alert.id}
                        className={`p-4 rounded-lg border-l-4 ${
                          alert.severity === "high"
                            ? "border-red-500 bg-red-50"
                            : alert.severity === "medium"
                              ? "border-yellow-500 bg-yellow-50"
                              : "border-blue-500 bg-blue-50"
                        }`}
                      >
                        <div className="flex justify-between">
                          <h3 className="font-bold">{alert.user}</h3>
                          <span className="text-xs text-gray-500">{alert.time}</span>
                        </div>
                        <p className="mt-1">{alert.message}</p>
                        <div className="mt-2 flex justify-end">
                          <button className="text-sm text-button hover:underline">Ver detalles</button>
                        </div>
                      </div>
                    ))}
                  </div>
                ) : (
                  <p className="text-gray-500 text-center py-4">No hay alertas activas</p>
                )}
              </div>

              <div className="p-4 border-t">
                <button className="text-button hover:underline text-sm flex items-center justify-center w-full">
                  Ver todas las alertas
                </button>
              </div>
            </div>

            {/* Activity Chart */}
            <div className="bg-white rounded-lg shadow-md overflow-hidden mt-8">
              <div className="p-4 border-b">
                <h2 className="text-lg font-bold">Actividad del Sistema</h2>
              </div>

              <div className="p-4">
                <div className="bg-gray-100 rounded-lg h-64 flex items-center justify-center">
                  <Activity size={48} className="text-gray-400" />
                </div>
                <div className="mt-4 grid grid-cols-2 gap-4">
                  <div>
                    <p className="text-sm text-gray-500">Rutas Completadas Hoy</p>
                    <p className="text-xl font-bold">24</p>
                  </div>
                  <div>
                    <p className="text-sm text-gray-500">Tiempo Medio de Ruta</p>
                    <p className="text-xl font-bold">18 min</p>
                  </div>
                  <div>
                    <p className="text-sm text-gray-500">Alertas Resueltas</p>
                    <p className="text-xl font-bold">12</p>
                  </div>
                  <div>
                    <p className="text-sm text-gray-500">Nuevos Usuarios</p>
                    <p className="text-xl font-bold">3</p>
                  </div>
                </div>
              </div>

              <div className="p-4 border-t">
                <button className="text-button hover:underline text-sm flex items-center justify-center w-full">
                  Ver estadísticas detalladas
                </button>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

