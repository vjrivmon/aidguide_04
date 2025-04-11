"use client"

import { useEffect } from "react"
import { useRouter } from "next/navigation"
import { useAuth } from "@/context/auth-context"
import { Bot, MapPin, Image, Bell, Map, User } from "lucide-react"
import Link from "next/link"

export default function Dashboard() {
  const { user } = useAuth()
  const router = useRouter()

  useEffect(() => {
    // Redirigir si no está autenticado o si no es un usuario regular
    if (!user) {
      router.push("/login")
    } else if (user.role !== "user") {
      // Redirigir según el rol
      if (user.role === "admin") {
        router.push("/admin/dashboard")
      } else if (user.role === "family") {
        router.push("/family")
      }
    }
  }, [user, router])

  if (!user || user.role !== "user") {
    return null
  }

  const menuItems = [
    {
      title: "Mi Robot",
      icon: <Bot size={36} className="text-button" />,
      description: "Controla y configura tu robot AidGuide",
      link: "/robot"
    },
    {
      title: "Mis Rutas",
      icon: <Map size={36} className="text-button" />,
      description: "Administra tus rutas guardadas",
      link: "/routes"
    },
    {
      title: "Lugares Frecuentes",
      icon: <MapPin size={36} className="text-button" />,
      description: "Gestiona tus lugares favoritos",
      link: "/places"
    },
    {
      title: "Galería de Imágenes",
      icon: <Image size={36} className="text-button" />,
      description: "Visualiza las imágenes capturadas por tu robot",
      link: "/robot-feed"
    },
    {
      title: "Notificaciones",
      icon: <Bell size={36} className="text-button" />,
      description: "Revisa tus alertas y notificaciones",
      link: "/notifications"
    },
    {
      title: "Mi Perfil",
      icon: <User size={36} className="text-button" />,
      description: "Gestiona tu perfil y preferencias",
      link: "/profile"
    }
  ]

  return (
    <div className="min-h-screen bg-background">
      <div className="container-custom py-8">
        <div className="bg-white rounded-lg shadow-md p-6 mb-8">
          <h1 className="text-2xl font-bold mb-4">Bienvenido, {user.nombre}</h1>
          <p className="text-gray-600">
            Desde este panel podrás controlar tu robot AidGuide y gestionar todas tus preferencias.
          </p>
        </div>

        {/* Menú de opciones */}
        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
          {menuItems.map((item, index) => (
            <Link key={index} href={item.link} className="bg-white rounded-lg shadow-md p-6 hover:shadow-lg transition-shadow">
              <div className="flex items-start">
                <div className="mr-4">
                  {item.icon}
                </div>
                <div>
                  <h2 className="text-xl font-semibold mb-2">{item.title}</h2>
                  <p className="text-gray-600">{item.description}</p>
                </div>
              </div>
            </Link>
          ))}
        </div>

        {/* Estado del robot */}
        <div className="bg-white rounded-lg shadow-md p-6 mt-8">
          <h2 className="text-xl font-semibold mb-4">Estado de tu Robot</h2>
          <div className="flex flex-col md:flex-row gap-6">
            <div className="flex-1 border rounded-lg p-4">
              <div className="flex items-center mb-2">
                <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
                <span className="font-medium">Conectado</span>
              </div>
              <p className="text-gray-600">Tu robot está actualmente conectado y funcionando correctamente.</p>
            </div>
            <div className="flex-1 border rounded-lg p-4">
              <div className="font-medium mb-2">Batería</div>
              <div className="w-full bg-gray-200 rounded-full h-4">
                <div className="bg-green-500 h-4 rounded-full" style={{ width: "75%" }}></div>
              </div>
              <p className="text-right text-sm mt-1">75%</p>
            </div>
            <div className="flex-1 border rounded-lg p-4">
              <div className="font-medium mb-2">Última ubicación</div>
              <p className="text-gray-600">Salón - Hace 5 minutos</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
} 