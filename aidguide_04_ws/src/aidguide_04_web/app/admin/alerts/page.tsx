"use client"

import { useState } from "react"
import Link from "next/link"
import {
  Search,
  Filter,
  Download,
  AlertTriangle,
  Battery,
  Wifi,
  PenToolIcon as Tool,
  User,
  Clock,
  CheckCircle,
  Bell,
} from "lucide-react"

export default function AdminAlerts() {
  const [filterStatus, setFilterStatus] = useState<string>("all")

  const alerts = [
    {
      id: "alert1",
      type: "battery",
      message: "Batería baja (15%)",
      user: "María García",
      robot: "AID-2023-001",
      time: "Hace 30 minutos",
      severity: "medium",
      status: "active",
    },
    {
      id: "alert2",
      type: "connection",
      message: "Pérdida de conexión",
      user: "Carlos Rodríguez",
      robot: "AID-2023-002",
      time: "Hace 2 horas",
      severity: "high",
      status: "active",
    },
    {
      id: "alert3",
      type: "maintenance",
      message: "Mantenimiento programado pendiente",
      user: "Ana Martínez",
      robot: "AID-2022-015",
      time: "Hace 3 días",
      severity: "low",
      status: "active",
    },
    {
      id: "alert4",
      type: "battery",
      message: "Batería crítica (5%)",
      user: "Javier López",
      robot: "AID-2023-003",
      time: "Hace 15 minutos",
      severity: "high",
      status: "active",
    },
    {
      id: "alert5",
      type: "connection",
      message: "Señal débil",
      user: "Laura Fernández",
      robot: "AID-2023-004",
      time: "Hace 45 minutos",
      severity: "low",
      status: "resolved",
    },
    {
      id: "alert6",
      type: "maintenance",
      message: "Error en sensor de proximidad",
      user: "Miguel Sánchez",
      robot: "AID-2022-010",
      time: "Hace 1 día",
      severity: "medium",
      status: "resolved",
    },
    {
      id: "alert7",
      type: "battery",
      message: "Batería agotada",
      user: "Carmen Gómez",
      robot: "AID-2022-008",
      time: "Hace 4 horas",
      severity: "high",
      status: "resolved",
    },
  ]

  const filteredAlerts = filterStatus === "all" ? alerts : alerts.filter((alert) => alert.status === filterStatus)

  const getAlertIcon = (type: string) => {
    switch (type) {
      case "battery":
        return <Battery className="text-red-500" size={20} />
      case "connection":
        return <Wifi className="text-yellow-500" size={20} />
      case "maintenance":
        return <Tool className="text-blue-500" size={20} />
      default:
        return <AlertTriangle className="text-gray-500" size={20} />
    }
  }

  const getSeverityClass = (severity: string) => {
    switch (severity) {
      case "high":
        return "bg-red-100 text-red-800 border-red-300"
      case "medium":
        return "bg-yellow-100 text-yellow-800 border-yellow-300"
      case "low":
        return "bg-blue-100 text-blue-800 border-blue-300"
      default:
        return "bg-gray-100 text-gray-800 border-gray-300"
    }
  }

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
            <Link href="/admin/alerts" className="py-4 px-6 border-b-2 border-button text-button font-medium">
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
            <h2 className="text-lg font-bold flex items-center">
              <Bell className="mr-2 text-button" />
              Gestión de Alertas
            </h2>

            <div className="flex flex-wrap gap-2">
              <div className="relative">
                <input
                  type="text"
                  placeholder="Buscar alerta..."
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
            </div>
          </div>

          <div className="p-4 border-b bg-gray-50">
            <div className="flex flex-wrap gap-2">
              <button
                className={`px-4 py-2 rounded-md ${filterStatus === "all" ? "bg-button text-white" : "bg-white border"}`}
                onClick={() => setFilterStatus("all")}
              >
                Todas ({alerts.length})
              </button>
              <button
                className={`px-4 py-2 rounded-md ${filterStatus === "active" ? "bg-button text-white" : "bg-white border"}`}
                onClick={() => setFilterStatus("active")}
              >
                Activas ({alerts.filter((a) => a.status === "active").length})
              </button>
              <button
                className={`px-4 py-2 rounded-md ${filterStatus === "resolved" ? "bg-button text-white" : "bg-white border"}`}
                onClick={() => setFilterStatus("resolved")}
              >
                Resueltas ({alerts.filter((a) => a.status === "resolved").length})
              </button>
            </div>
          </div>

          <div className="p-4">
            <div className="grid grid-cols-1 gap-4">
              {filteredAlerts.length > 0 ? (
                filteredAlerts.map((alert) => (
                  <div key={alert.id} className={`p-4 rounded-lg border ${getSeverityClass(alert.severity)}`}>
                    <div className="flex flex-col sm:flex-row justify-between">
                      <div className="flex items-start">
                        <div className="mr-3">{getAlertIcon(alert.type)}</div>
                        <div>
                          <h3 className="font-bold">{alert.message}</h3>
                          <div className="flex flex-wrap gap-x-4 mt-1 text-sm">
                            <div className="flex items-center">
                              <User size={16} className="mr-1 text-gray-500" />
                              <span>{alert.user}</span>
                            </div>
                            <div className="flex items-center">
                              <Tool size={16} className="mr-1 text-gray-500" />
                              <span>{alert.robot}</span>
                            </div>
                            <div className="flex items-center">
                              <Clock size={16} className="mr-1 text-gray-500" />
                              <span>{alert.time}</span>
                            </div>
                          </div>
                        </div>
                      </div>

                      <div className="flex items-center mt-3 sm:mt-0">
                        {alert.status === "active" ? (
                          <div className="flex gap-2">
                            <button className="px-3 py-1 bg-green-500 text-white rounded-md flex items-center text-sm">
                              <CheckCircle size={16} className="mr-1" />
                              Resolver
                            </button>
                            <button className="px-3 py-1 bg-blue-500 text-white rounded-md flex items-center text-sm">
                              <User size={16} className="mr-1" />
                              Asignar
                            </button>
                          </div>
                        ) : (
                          <span className="flex items-center text-green-600">
                            <CheckCircle size={16} className="mr-1" />
                            Resuelta
                          </span>
                        )}
                      </div>
                    </div>

                    {alert.status === "active" && (
                      <div className="mt-3 pt-3 border-t border-gray-200 flex justify-between items-center">
                        <div className="text-sm">
                          <span className="font-medium">Severidad: </span>
                          <span
                            className={`${
                              alert.severity === "high"
                                ? "text-red-600"
                                : alert.severity === "medium"
                                  ? "text-yellow-600"
                                  : "text-blue-600"
                            }`}
                          >
                            {alert.severity === "high" ? "Alta" : alert.severity === "medium" ? "Media" : "Baja"}
                          </span>
                        </div>
                        <div className="flex gap-2">
                          <button className="text-sm text-button hover:underline">Ver detalles</button>
                          <button className="text-sm text-button hover:underline">Historial</button>
                        </div>
                      </div>
                    )}
                  </div>
                ))
              ) : (
                <div className="text-center py-8">
                  <div className="mx-auto w-16 h-16 bg-gray-100 rounded-full flex items-center justify-center mb-4">
                    <Bell size={32} className="text-gray-400" />
                  </div>
                  <h3 className="text-lg font-medium text-gray-900 mb-1">No hay alertas</h3>
                  <p className="text-gray-500">No se encontraron alertas con los filtros seleccionados.</p>
                </div>
              )}
            </div>
          </div>

          {filteredAlerts.length > 0 && (
            <div className="p-4 border-t flex justify-between items-center">
              <p className="text-sm text-gray-500">
                Mostrando {filteredAlerts.length} de {alerts.length} alertas
              </p>
              <div className="flex gap-2">
                <button className="px-3 py-1 border rounded-md text-sm">Anterior</button>
                <button className="px-3 py-1 bg-button text-white rounded-md text-sm">1</button>
                <button className="px-3 py-1 border rounded-md text-sm">Siguiente</button>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  )
}

