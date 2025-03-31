"use client"

import { useState } from "react"
import Image from "next/image"
import { Camera, TrafficCone, Signpost, Bus, Users, Footprints, Wrench, Video } from "lucide-react"
import { useAuth } from "@/context/auth-context"

export default function RobotFeed() {
  const { user } = useAuth()
  const [selectedCategory, setSelectedCategory] = useState<string | null>(null)

  const categories = [
    { id: "live", name: "En vivo", icon: Video },
    { id: "traffic", name: "Señales de tráfico", icon: TrafficCone },
    { id: "people", name: "Personas", icon: Users },
    { id: "bus", name: "Paradas de autobús", icon: Bus },
    { id: "crosswalk", name: "Pasos de peatones", icon: Footprints },
    { id: "construction", name: "Obras", icon: Wrench },
    { id: "road-closed", name: "Calles cortadas", icon: Signpost },
  ]

  return (
    <div className="container-custom py-14">
      {/* Título y subtítulo */}
      <div className="text-center mb-12">
        <h1 className="text-4xl md:text-4xl font-bold mb-4">
          Imágenes captadas por el robot
        </h1>
        <h2 className="text-2xl text-text">
          Vista en tiempo real y detecciones
        </h2>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
        {/* Panel izquierdo - Categorías */}
        <div className="bg-white rounded-lg shadow-md p-6">
          <h2 className="text-2xl font-bold text-button mb-6">Categorías</h2>
          <nav className="space-y-2">
            {categories.map((category) => {
              const Icon = category.icon
              return (
                <button
                  key={category.id}
                  onClick={() => setSelectedCategory(category.id)}
                  className={`w-full flex items-center px-4 py-3 rounded-lg transition-colors ${
                    selectedCategory === category.id
                      ? "bg-button text-white"
                      : "text-text hover:bg-gray-50"
                  }`}
                >
                  <Icon size={20} className="mr-3" />
                  {category.name}
                </button>
              )
            })}
          </nav>
        </div>

        {/* Panel central - Contenido */}
        <div className="md:col-span-2 bg-white rounded-lg shadow-md p-6">
          {selectedCategory === "live" ? (
            <div>
              <h2 className="text-2xl font-bold text-button mb-6">Vista en tiempo real</h2>
              <div className="relative aspect-video bg-gray-900 rounded-lg overflow-hidden">
                <div className="absolute inset-0 flex items-center justify-center">
                  <div className="text-white text-center">
                    <Video size={48} className="mx-auto mb-4" />
                    <p className="text-lg">Vista en tiempo real del robot</p>
                    <p className="text-sm text-gray-400">Conectando a la cámara...</p>
                  </div>
                </div>
              </div>
            </div>
          ) : (
            <div>
              <h2 className="text-2xl font-bold text-button mb-6">
                {categories.find(c => c.id === selectedCategory)?.name || "Imágenes detectadas"}
              </h2>
              <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
                {/* Grid de imágenes */}
                {[1, 2, 3, 4].map((index) => (
                  <div key={index} className="bg-gray-50 p-4 rounded-lg">
                    <div className="relative h-48 bg-gray-200 rounded-lg overflow-hidden">
                      <Image
                        src="/placeholder.svg?height=200&width=200"
                        alt={`Imagen ${index}`}
                        fill
                        className="object-cover"
                      />
                    </div>
                    <p className="text-sm text-gray-500 mt-2">Detectado: 15/03/2024, 10:30</p>
                  </div>
                ))}
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  )
} 