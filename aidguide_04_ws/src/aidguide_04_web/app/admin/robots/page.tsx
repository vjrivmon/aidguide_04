"use client"

import { useState } from "react"
import Link from "next/link"
import {
  Search,
  Filter,
  Download,
  ChevronDown,
  ChevronUp,
  Settings,
  RefreshCw,
  AlertTriangle,
  Plus,
} from "lucide-react"

export default function AdminRobots() {
  const [expandedRobot, setExpandedRobot] = useState<string | null>(null)

  const toggleRobotExpand = (robotId: string) => {
    if (expandedRobot === robotId) {
      setExpandedRobot(null)
    } else {
      setExpandedRobot(robotId)
    }
  }

  const robots = [
    {
      id: "robot1",
      serialNumber: "AID-2023-001",
      model: "AidGuide Pro",
      status: "online",
      batteryLevel: 85,
      lastMaintenance: "15/01/2023",
      nextMaintenance: "15/04/2023",
      assignedTo: "María García",
      location: "Valencia, España",
      firmwareVersion: "v2.3.5",
      lastUpdate: "10/02/2023",
    },
    {
      id: "robot2",
      serialNumber: "AID-2023-002",
      model: "AidGuide Pro",
      status: "offline",
      batteryLevel: 20,
      lastMaintenance: "20/12/2022",
      nextMaintenance: "20/03/2023",
      assignedTo: "Carlos Rodríguez",
      location: "Barcelona, España",
      firmwareVersion: "v2.3.4",
      lastUpdate: "05/01/2023",
    },
    {
      id: "robot3",
      serialNumber: "AID-2022-015",
      model: "AidGuide Standard",
      status: "maintenance",
      batteryLevel: 0,
      lastMaintenance: "10/03/2023",
      nextMaintenance: "10/06/2023",
      assignedTo: "Ana Martínez",
      location: "Centro de Servicio, Madrid",
      firmwareVersion: "v2.2.8",
      lastUpdate: "15/12/2022",
    },
    {
      id: "robot4",
      serialNumber: "AID-2023-003",
      model: "AidGuide Pro",
      status: "online",
      batteryLevel: 72,
      lastMaintenance: "05/02/2023",
      nextMaintenance: "05/05/2023",
      assignedTo: "Javier López",
      location: "Sevilla, España",
      firmwareVersion: "v2.3.5",
      lastUpdate: "05/02/2023",
    },
    {
      id: "robot5",
      serialNumber: "AID-2023-004",
      model: "AidGuide Pro+",
      status: "online",
      batteryLevel: 95,
      lastMaintenance: "25/02/2023",
      nextMaintenance: "25/05/2023",
      assignedTo: "Laura Fernández",
      location: "Valencia, España",
      firmwareVersion: "v2.3.5",
      lastUpdate: "25/02/2023",
    },
    {
      id: "robot6",
      serialNumber: "AID-2022-010",
      model: "AidGuide Standard",
      status: "offline",
      batteryLevel: 0,
      lastMaintenance: "10/11/2022",
      nextMaintenance: "10/02/2023",
      assignedTo: "Miguel Sánchez",
      location: "Alicante, España",
      firmwareVersion: "v2.2.7",
      lastUpdate: "10/11/2022",
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
            <Link href="/admin/robots" className="py-4 px-6 border-b-2 border-button text-button font-medium">
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
            <h2 className="text-lg font-bold">Gestión de Robots</h2>

            <div className="flex flex-wrap gap-2">
              <div className="relative">
                <input
                  type="text"
                  placeholder="Buscar robot..."
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
                Añadir Robot
              </button>
            </div>
          </div>

          <div className="overflow-x-auto">
            <table className="w-full">
              <thead>
                <tr className="bg-gray-50">
                  <th className="text-left py-3 px-4 font-medium">Robot</th>
                  <th className="text-left py-3 px-4 font-medium">Estado</th>
                  <th className="text-left py-3 px-4 font-medium">Batería</th>
                  <th className="text-left py-3 px-4 font-medium">Usuario Asignado</th>
                  <th className="text-left py-3 px-4 font-medium">Acciones</th>
                </tr>
              </thead>
              <tbody>
                {robots.map((robot) => (
                  <>
                    <tr key={robot.id} className="border-b hover:bg-gray-50">
                      <td className="py-3 px-4">
                        <div>
                          <p className="font-medium">{robot.serialNumber}</p>
                          <p className="text-sm text-gray-500">{robot.model}</p>
                        </div>
                      </td>
                      <td className="py-3 px-4">
                        <span
                          className={`inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium ${
                            robot.status === "online"
                              ? "bg-green-100 text-green-800"
                              : robot.status === "offline"
                                ? "bg-red-100 text-red-800"
                                : "bg-yellow-100 text-yellow-800"
                          }`}
                        >
                          {robot.status === "online"
                            ? "En línea"
                            : robot.status === "offline"
                              ? "Desconectado"
                              : "Mantenimiento"}
                        </span>
                      </td>
                      <td className="py-3 px-4">
                        <div className="flex items-center">
                          <div className="w-16 bg-gray-200 rounded-full h-2.5 mr-2">
                            <div
                              className={`h-2.5 rounded-full ${
                                robot.batteryLevel > 60
                                  ? "bg-green-600"
                                  : robot.batteryLevel > 20
                                    ? "bg-yellow-500"
                                    : "bg-red-600"
                              }`}
                              style={{ width: `${robot.batteryLevel}%` }}
                            ></div>
                          </div>
                          <span className="text-sm font-medium">{robot.batteryLevel}%</span>
                        </div>
                      </td>
                      <td className="py-3 px-4 text-sm">{robot.assignedTo}</td>
                      <td className="py-3 px-4">
                        <div className="flex items-center gap-2">
                          <button
                            className="text-gray-500 hover:text-button"
                            onClick={() => toggleRobotExpand(robot.id)}
                            aria-label={expandedRobot === robot.id ? "Colapsar detalles" : "Expandir detalles"}
                          >
                            {expandedRobot === robot.id ? <ChevronUp size={18} /> : <ChevronDown size={18} />}
                          </button>
                          <button className="text-blue-500 hover:text-blue-700">
                            <Settings size={18} />
                          </button>
                          <button className="text-green-500 hover:text-green-700">
                            <RefreshCw size={18} />
                          </button>
                          {robot.nextMaintenance < new Date().toLocaleDateString("es-ES") && (
                            <button className="text-yellow-500 hover:text-yellow-700">
                              <AlertTriangle size={18} />
                            </button>
                          )}
                        </div>
                      </td>
                    </tr>
                    {expandedRobot === robot.id && (
                      <tr className="bg-gray-50">
                        <td colSpan={5} className="py-4 px-6">
                          <div className="grid grid-cols-1 sm:grid-cols-3 gap-4">
                            <div>
                              <p className="text-sm text-gray-500">Ubicación</p>
                              <p className="font-medium">{robot.location}</p>
                            </div>
                            <div>
                              <p className="text-sm text-gray-500">Versión de Firmware</p>
                              <p className="font-medium">{robot.firmwareVersion}</p>
                            </div>
                            <div>
                              <p className="text-sm text-gray-500">Última Actualización</p>
                              <p className="font-medium">{robot.lastUpdate}</p>
                            </div>
                            <div>
                              <p className="text-sm text-gray-500">Último Mantenimiento</p>
                              <p className="font-medium">{robot.lastMaintenance}</p>
                            </div>
                            <div>
                              <p className="text-sm text-gray-500">Próximo Mantenimiento</p>
                              <p
                                className={`font-medium ${
                                  robot.nextMaintenance < new Date().toLocaleDateString("es-ES") ? "text-red-600" : ""
                                }`}
                              >
                                {robot.nextMaintenance}
                                {robot.nextMaintenance < new Date().toLocaleDateString("es-ES") && " (Atrasado)"}
                              </p>
                            </div>
                          </div>
                          <div className="mt-4 flex gap-2">
                            <button className="text-sm text-button hover:underline">Ver historial completo</button>
                            <button className="text-sm text-button hover:underline">Programar mantenimiento</button>
                            <button className="text-sm text-button hover:underline">Actualizar firmware</button>
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
            <p className="text-sm text-gray-500">Mostrando 6 de 28 robots</p>
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

