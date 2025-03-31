"use client"

import { useState } from "react"
import Link from "next/link"
import { Search, Filter, Download, ChevronDown, ChevronUp, Edit, Trash2, UserPlus } from "lucide-react"

export default function AdminUsers() {
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
    {
      id: "user5",
      name: "Laura Fernández",
      email: "laura.fernandez@example.com",
      status: "active",
      lastActive: "Hace 1 hora",
      robotStatus: "online",
      batteryLevel: 90,
      location: "Valencia, España",
      registrationDate: "05/02/2023",
    },
    {
      id: "user6",
      name: "Miguel Sánchez",
      email: "miguel.sanchez@example.com",
      status: "inactive",
      lastActive: "Hace 10 días",
      robotStatus: "offline",
      batteryLevel: 0,
      location: "Alicante, España",
      registrationDate: "18/12/2022",
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
            <Link href="/admin/users" className="py-4 px-6 border-b-2 border-button text-button font-medium">
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
        <div className="bg-white rounded-lg shadow-md overflow-hidden">
          <div className="p-4 border-b flex flex-col sm:flex-row justify-between items-start sm:items-center gap-4">
            <h2 className="text-lg font-bold">Gestión de Usuarios</h2>

            <div className="flex flex-wrap gap-2">
              <div className="relative">
                <input
                  type="text"
                  placeholder="Buscar usuario..."
                  className="pl-10 pr-4 py-2 border rounded-md focus:outline-none focus:ring-2 focus:ring-button"
                />
                <Search className="absolute left-3 top-1/2 transform -translate-y-1/2 text-gray-400" size={18} />
              </div>

              <button className="p-2 rounded-md hover:bg-gray-100 border">
                <Filter size={18} />
              </button>

              <button className="p-2 rounded-md hover:bg-gray-100 border">
                <Download size={18} />
              </button>

              <button className="btn-primary flex items-center">
                <UserPlus size={18} className="mr-2" />
                Añadir Usuario
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
                  <th className="text-left py-3 px-4 font-medium">Acciones</th>
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
                          <button className="text-blue-500 hover:text-blue-700">
                            <Edit size={18} />
                          </button>
                          <button className="text-red-500 hover:text-red-700">
                            <Trash2 size={18} />
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
            <p className="text-sm text-gray-500">Mostrando 6 de 32 usuarios</p>
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
    </div>
  )
}

