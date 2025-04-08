"use client"

import { useEffect, useRef, useState } from 'react'
import L from 'leaflet'
import { useMap } from 'react-leaflet'
import DummyMapGenerator from '@/app/components/DummyMapGenerator'

interface RosMapProps {
  mapUrl: string
  yamlUrl: string
}

interface MapInfo {
  image: string
  resolution: number
  origin: [number, number, number]
  negate: number
  occupied_thresh: number
  free_thresh: number
}

// Componente para cargar y mostrar un mapa PGM de ROS2 en un mapa Leaflet
export default function RosMapLoader({ mapUrl, yamlUrl }: RosMapProps) {
  const map = useMap()
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const imageLayerRef = useRef<L.ImageOverlay | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [dummyMapUrl, setDummyMapUrl] = useState<string | null>(null)
  const [yamlData, setYamlData] = useState<MapInfo | null>(null)
  const [useDummyMap, setUseDummyMap] = useState(false)

  // Función para manejar cuando se genera el mapa dummy
  const handleDummyMapGenerated = (dataUrl: string) => {
    setDummyMapUrl(dataUrl)
  }

  // Cargar el archivo YAML
  useEffect(() => {
    const loadYaml = async () => {
      try {
        // Cargar los datos del mapa directamente desde nuestro YAML
        const defaultYamlData: MapInfo = {
          image: mapUrl,
          resolution: 0.05,
          origin: [-8.16, -8.97, 0],
          negate: 0,
          occupied_thresh: 0.65,
          free_thresh: 0.25
        }
        
        setYamlData(defaultYamlData)
      } catch (err) {
        console.error('Error al configurar los datos del mapa:', err)
        setError(`Error al configurar el mapa: ${err instanceof Error ? err.message : String(err)}`)
        setUseDummyMap(true)
      }
    }
    
    loadYaml()
  }, [mapUrl])

  // Este useEffect se encarga de cargar la imagen del mapa
  useEffect(() => {
    if (!yamlData) return

    const canvas = document.createElement('canvas')
    canvasRef.current = canvas
    
    // Crear un mapa de prueba si estamos usando el mapa dummy
    if (useDummyMap) {
      console.log("Se usará el mapa dummy porque el mapa real no pudo cargarse")
      return
    }
    
    // Intentar cargar directamente el mapa PGM/PNG
    const loadImage = () => {
      setLoading(true)
      
      const img = new Image()
      img.crossOrigin = 'anonymous' // Para evitar problemas CORS
      
      img.onload = () => {
        console.log("✅ Imagen cargada correctamente:", img.width, "x", img.height)
        
        // Dibujar la imagen en el canvas
        canvas.width = img.width
        canvas.height = img.height
        
        const ctx = canvas.getContext('2d')
        if (!ctx) {
          setError('No se pudo obtener el contexto del canvas')
          setLoading(false)
          setUseDummyMap(true)
          return
        }
        
        ctx.drawImage(img, 0, 0)
        
        // Generar la URL de datos
        const dataUrl = canvas.toDataURL('image/png')
        
        // Limpiar cualquier capa existente
        if (imageLayerRef.current) {
          imageLayerRef.current.remove()
        }
        
        // Calcular las dimensiones del mapa
        const mapWidth = img.width * yamlData.resolution
        const mapHeight = img.height * yamlData.resolution
        
        // Crear los límites del mapa
        const bounds = L.latLngBounds(
          [yamlData.origin[1], yamlData.origin[0]],
          [yamlData.origin[1] + mapHeight, yamlData.origin[0] + mapWidth]
        )
        
        // Crear y añadir la capa del mapa
        const imageOverlay = L.imageOverlay(dataUrl, bounds, {
          opacity: 0.7,
          zIndex: 10
        }).addTo(map)
        
        imageLayerRef.current = imageOverlay
        
        // Ajustar el mapa para mostrar toda la imagen
        map.fitBounds(bounds)
        
        setLoading(false)
      }
      
      img.onerror = (e) => {
        console.error("❌ Error al cargar la imagen del mapa:", e)
        setError('Error al cargar el mapa. Usando mapa de demostración.')
        setUseDummyMap(true)
        setLoading(false)
      }
      
      // Iniciar la carga de la imagen
      console.log("🔄 Intentando cargar la imagen desde:", mapUrl)
      img.src = mapUrl
    }
    
    loadImage()
    
    return () => {
      // Limpieza al desmontar
      if (imageLayerRef.current) {
        imageLayerRef.current.remove()
      }
    }
  }, [map, mapUrl, yamlData, useDummyMap])

  // Procesar el mapa dummy cuando esté disponible
  useEffect(() => {
    if (!useDummyMap || !dummyMapUrl || !yamlData) return
    
    console.log("🔄 Cargando mapa de demostración...")
    
    // Limpiar cualquier capa existente
    if (imageLayerRef.current) {
      imageLayerRef.current.remove()
    }
    
    // Dimensiones del mapa dummy
    const mapWidth = 400 * yamlData.resolution
    const mapHeight = 400 * yamlData.resolution
    
    // Crear los límites
    const bounds = L.latLngBounds(
      [yamlData.origin[1], yamlData.origin[0]],
      [yamlData.origin[1] + mapHeight, yamlData.origin[0] + mapWidth]
    )
    
    // Crear y añadir la capa
    const imageOverlay = L.imageOverlay(dummyMapUrl, bounds, {
      opacity: 0.7,
      zIndex: 10
    }).addTo(map)
    
    imageLayerRef.current = imageOverlay
    
    // Ajustar el mapa
    map.fitBounds(bounds)
    
    setLoading(false)
    console.log("✅ Mapa de demostración cargado correctamente")
  }, [map, dummyMapUrl, yamlData, useDummyMap])
  
  return (
    <>
      {loading && (
        <div style={{ 
          position: 'absolute', 
          top: '50%', 
          left: '50%', 
          transform: 'translate(-50%, -50%)',
          background: 'rgba(255,255,255,0.8)',
          padding: '10px',
          borderRadius: '5px',
          zIndex: 1000
        }}>
          <p>Cargando mapa...</p>
        </div>
      )}
      {error && (
        <div style={{ 
          position: 'absolute', 
          bottom: '10px', 
          left: '50%', 
          transform: 'translateX(-50%)',
          background: 'rgba(255,0,0,0.7)',
          color: 'white',
          padding: '5px 10px',
          borderRadius: '5px',
          fontSize: '12px',
          zIndex: 1000
        }}>
          {error}
        </div>
      )}
      <DummyMapGenerator width={400} height={400} onMapGenerated={handleDummyMapGenerated} />
    </>
  )
} 