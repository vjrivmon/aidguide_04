"use client"

import { useState } from "react"
import Link from "next/link"
import { Search, Filter, Download, MapPin, Clock, User, Calendar, ChevronDown, ChevronUp, Plus } from "lucide-react"

export default function AdminRoutes() {
  const [expandedRoute, setExpandedRoute] = useState<string | null>(null)

  const toggleRouteExpand = (routeId: string) => {
    if (expandedRoute === routeId) {
      setExpandedRoute(null)
    } else {
      setExpandedRoute(routeId)
    }
  }

  const routes = [
    {
      id: "route1",
      name: "Casa - Trabajo",
      user: "María García",
      date: "15/03/2023",
      time: "08:30",
      duration: "22 min",
      distance: "1.5 km",
      status: "completed",
      waypoints: ["Calle Principal 123", "Parque Central", "Avenida Comercial 45", "Edificio Empresarial"],
    },
    {
      id: "route2",
      name: "Trabajo - Casa",
      user: "María García",
      date: "15/03/2023",
      time: "17:45",
      duration: "25 min",
      distance: "1.6 km",
      status: "completed",
      waypoints: ["Edificio Empresarial", "Plaza Mayor", "Calle Comercio", "Calle Principal 123"],
    },
    {
      id: "route3",
      name: "Casa - Supermercado",
      user: "Carlos Rodríguez",
      date: "14/03/2023",
      time: "10:15",
      duration: "15 min",
      distance: "0.8 km",
      status: "completed",
      waypoints: ["Calle Barcelona 45", "Avenida Central", "Supermercado Mercadona"],
    },
    {
      id: "route4",
      name: "Centro Médico - Casa",
      user: "Ana Martínez",
      date: "10/03/2023",
      time: "11:30",
      duration: "30 min",
      distance: "2.3 km",
      status: "completed",
      waypoints: ["Centro Médico Municipal", "Parque Norte", "Avenida Principal", "Calle Madrid 78"],
    },
    {
      id: "route5",
      name: "Casa - Gimnasio",
      user: "Javier López",
      date: "15/03/2023",
      time: "18:00",
      duration: "12 min",
      distance: "0.7 km",
      status: "completed",
      waypoints: ["Calle Sevilla 12", "Plaza del Ayuntamiento", "Gimnasio Fitness Center"],
    },
    {
      id: "route6",
      name: "Trabajo - Restaurante",
      user: "Laura Fernández",
      date: "15/03/2023",
      time: "14:00",
      duration: "10 min",
      distance: "0.5 km",
      status: "completed",
      waypoints: ["Oficina Central", "Calle Mayor", "Restaurante La Plaza"],
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
            <Link href="/admin/users" className="py-4 px-6 text-gray-500 hover:text-button font-medium">
              Usuarios
            </Link>
            <Link href="/admin/robots" className="py-4 px-6 text-gray-500 hover:text-button font-medium">
              Robots
            </Link>
            <Link href="/admin/routes" className="py-4 px-6 border-b-2 border-button text-button font-medium">
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
            <h2 className="text-lg font-bold">Historial de Rutas</h2>

            <div className="flex flex-wrap gap-2">
              <div className="relative">
                <input
                  type="text"
                  placeholder="Buscar ruta..."
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
                <Plus size={18} className="mr-2" />
                Nueva Ruta
              </button>
            </div>
          </div>

          <div className="overflow-x-auto">
            <table className="w-full">
              <thead>
                <tr className="bg-gray-50">
                  <th className="text-left py-3 px-4 font-medium">Ruta</th>
                  <th className="text-left py-3 px-4 font-medium">Usuario</th>
                  <th className="text-left py-3 px-4 font-medium">Fecha y Hora</th>
                  <th className="text-left py-3 px-4 font-medium">Duración</th>
                  <th className="text-left py-3 px-4 font-medium">Estado</th>
                  <th className="text-left py-3 px-4 font-medium">Detalles</th>
                </tr>
              </thead>
              <tbody>
                {routes.map((route) => (
                  <>
                    <tr key={route.id} className="border-b hover:bg-gray-50">
                      <td className="py-3 px-4">
                        <div className="flex items-center">
                          <MapPin size={18} className="text-button mr-2" />
                          <span>{route.name}</span>
                        </div>
                      </td>
                      <td className="py-3 px-4">
                        <div className="flex items-center">
                          <User size={18} className="text-gray-400 mr-2" />
                          <span>{route.user}</span>
                        </div>
                      </td>
                      <td className="py-3 px-4">
                        <div className="flex items-center">
                          <Calendar size={18} className="text-gray-400 mr-2" />
                          <span>
                            {route.date}, {route.time}
                          </span>
                        </div>
                      </td>
                      <td className="py-3 px-4">
                        <div className="flex items-center">
                          <Clock size={18} className="text-gray-400 mr-2" />
                          <span>{route.duration}</span>
                        </div>
                      </td>
                      <td className="py-3 px-4">
                        <span
                          className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
                            route.status === "completed"
                              ? "bg-green-100 text-green-800"
                              : route.status === "in-progress"
                                ? "bg-blue-100 text-blue-800"
                                : "bg-yellow-100 text-yellow-800"
                          }`}
                        >
                          {route.status === "completed"
                            ? "Completada"
                            : route.status === "in-progress"
                              ? "En progreso"
                              : "Programada"}
                        </span>
                      </td>
                      <td className="py-3 px-4">
                        <button
                          className="text-button hover:underline flex items-center"
                          onClick={() => toggleRouteExpand(route.id)}
                        >
                          {expandedRoute === route.id ? (
                            <>
                              <ChevronUp size={18} className="mr-1" />
                              Ocultar
                            </>
                          ) : (
                            <>
                              <ChevronDown size={18} className="mr-1" />
                              Ver
                            </>
                          )}
                        </button>
                      </td>
                    </tr>
                    {expandedRoute === route.id && (
                      <tr className="bg-gray-50">
                        <td colSpan={6} className="py-4 px-6">
                          <div>
                            <h3 className="font-bold mb-2">Detalles de la Ruta</h3>
                            <div className="grid grid-cols-1 sm:grid-cols-3 gap-4 mb-4">
                              <div>
                                <p className="text-sm text-gray-500">Distancia</p>
                                <p className="font-medium">{route.distance}</p>
                              </div>
                              <div>
                                <p className="text-sm text-gray-500">Duración</p>
                                <p className="font-medium">{route.duration}</p>
                              </div>
                              <div>
                                <p className="text-sm text-gray-500">Usuario</p>
                                <p className="font-medium">{route.user}</p>
                              </div>
                            </div>

                            <h4 className="font-medium mb-2">Puntos de ruta</h4>
                            <ul className="space-y-2 mb-4">
                              {route.waypoints.map((waypoint, index) => (
                                <li key={index} className="flex items-center">
                                  <div className="bg-button text-white w-6 h-6 rounded-full flex items-center justify-center mr-3 flex-shrink-0">
                                    {index + 1}
                                  </div>
                                  <span>{waypoint}</span>
                                </li>
                              ))}
                            </ul>

                            <div className="bg-gray-200 rounded-lg h-40 flex items-center justify-center mb-4">
                              <p className="text-gray-500">[Aquí se mostraría el mapa de la ruta]</p>
                            </div>

                            <div className="flex gap-2">
                              <button className="text-sm text-button hover:underline">Ver mapa completo</button>
                              <button className="text-sm text-button hover:underline">Exportar datos</button>
                              <button className="text-sm text-button hover:underline">Generar informe</button>
                            </div>
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
            <p className="text-sm text-gray-500">Mostrando 6 de 156 rutas</p>
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

