"use client"

import { useEffect, useState } from 'react'
import { 
  MapContainer, 
  TileLayer, 
  Marker, 
  Popup, 
  Polyline, 
  useMap, 
  ZoomControl,
  LayersControl
} from 'react-leaflet'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'

// Corregir el problema de los íconos de Leaflet en Next.js
import icon from 'leaflet/dist/images/marker-icon.png'
import iconShadow from 'leaflet/dist/images/marker-shadow.png'
import RosMapLoader from '@/app/components/RosMapLoader'

// Definir las propiedades del componente
interface RouteMapProps {
  waypoints: string[]
  routeName: string
  pgmMapUrl?: string
  pgmYamlUrl?: string
}

// Componente para ajustar la vista del mapa
function FitBounds({ waypoints }: { waypoints: [number, number][] }) {
  const map = useMap()
  
  useEffect(() => {
    if (waypoints.length > 0) {
      const bounds = L.latLngBounds(waypoints.map(point => L.latLng(point[0], point[1])))
      map.fitBounds(bounds, { padding: [30, 30] })
    }
  }, [map, waypoints])
  
  return null
}

// Botón de pantalla completa personalizado
function FullscreenControl() {
  const map = useMap()
  
  const toggleFullscreen = () => {
    const container = map.getContainer()
    
    if (!document.fullscreenElement) {
      container.requestFullscreen().catch(err => {
        console.error(`Error al intentar entrar en modo pantalla completa: ${err.message}`)
      })
    } else {
      document.exitFullscreen()
    }
  }
  
  useEffect(() => {
    // Crear un botón de control personalizado
    const fullscreenControl = L.Control.extend({
      options: {
        position: 'bottomright'
      },
      
      onAdd: function() {
        const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control')
        const button = L.DomUtil.create('a', '', container)
        
        button.innerHTML = '⛶'
        button.title = 'Pantalla completa'
        button.style.fontWeight = 'bold'
        button.style.fontSize = '18px'
        button.style.lineHeight = '30px'
        button.style.textAlign = 'center'
        button.style.width = '30px'
        button.style.height = '30px'
        button.style.backgroundColor = 'white'
        button.style.display = 'block'
        button.style.cursor = 'pointer'
        
        L.DomEvent.on(button, 'click', L.DomEvent.stopPropagation)
          .on(button, 'click', L.DomEvent.preventDefault)
          .on(button, 'click', toggleFullscreen)
        
        return container
      }
    })
    
    // Añadir el control al mapa
    map.addControl(new fullscreenControl())
    
    return () => {
      // No es necesario limpiar en este caso ya que Leaflet lo maneja
    }
  }, [map])
  
  return null
}

// Simular la conversión de direcciones a coordenadas
const simulateGeocode = (address: string): [number, number] => {
  // En una aplicación real, aquí llamarías a un servicio de geocodificación
  // Por ahora, generamos coordenadas alrededor de Madrid
  const madridLat = 40.416775
  const madridLng = -3.703790
  
  // Generar coordenadas aleatorias cercanas a Madrid basadas en el hash de la dirección
  const hash = address.split('').reduce((acc, char) => acc + char.charCodeAt(0), 0)
  const lat = madridLat + (hash % 20 - 10) / 100
  const lng = madridLng + ((hash * 13) % 20 - 10) / 100
  
  return [lat, lng]
}

// Componente principal
export default function RouteMap({ 
  waypoints, 
  routeName, 
  pgmMapUrl = '/aidguide_04_map.png', 
  pgmYamlUrl = '/aidguide_04_map.yaml' 
}: RouteMapProps) {
  const [coordinates, setCoordinates] = useState<[number, number][]>([])
  const [isMapReady, setIsMapReady] = useState(false)
  const [useRosMap, setUseRosMap] = useState(true)
  
  // Configurar los iconos de Leaflet
  useEffect(() => {
    if (typeof window !== 'undefined') {
      // Solución para el problema de iconos en Next.js
      const DefaultIcon = L.icon({
        iconUrl: icon.src,
        shadowUrl: iconShadow.src,
        iconSize: [25, 41],
        iconAnchor: [12, 41],
        popupAnchor: [1, -34],
        shadowSize: [41, 41]
      })
      L.Marker.prototype.options.icon = DefaultIcon
      setIsMapReady(true)
    }
  }, [])

  // Convertir waypoints a coordenadas
  useEffect(() => {
    if (waypoints.length > 0) {
      const coords = waypoints.map(point => simulateGeocode(point))
      setCoordinates(coords)
    }
  }, [waypoints])

  // Si no hay coordenadas, mostrar un mensaje
  if (coordinates.length === 0) {
    return (
      <div className="h-full flex items-center justify-center">
        <p className="text-gray-500">Cargando coordenadas...</p>
      </div>
    )
  }

  return (
    <div className="h-full w-full">
      {isMapReady && (
        <MapContainer
          center={coordinates[0] || [40.416775, -3.703790]}
          zoom={13}
          style={{ height: '100%', width: '100%' }}
          zoomControl={false} // Desactivamos el control de zoom predeterminado
        >
          <LayersControl position="topright">
            <LayersControl.BaseLayer checked={!useRosMap} name="OpenStreetMap">
              <TileLayer
                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
              />
            </LayersControl.BaseLayer>
            
            <LayersControl.BaseLayer name="Satélite">
              <TileLayer
                attribution='&copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community'
                url="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
              />
            </LayersControl.BaseLayer>
            
            <LayersControl.Overlay checked={useRosMap} name="Mapa ROS2">
              <RosMapLoader mapUrl={pgmMapUrl} yamlUrl={pgmYamlUrl} />
            </LayersControl.Overlay>
          </LayersControl>
          
          {/* Agregamos el control de zoom en la posición inferior derecha */}
          <ZoomControl position="bottomright" />
          
          {/* Botón de pantalla completa */}
          <FullscreenControl />
          
          {/* Mostramos los marcadores */}
          {coordinates.map((position, index) => (
            <Marker key={`marker-${index}`} position={position}>
              <Popup>
                <strong>{index + 1}. {waypoints[index]}</strong>
              </Popup>
            </Marker>
          ))}
          
          {/* Dibujamos la línea de la ruta */}
          <Polyline 
            positions={coordinates} 
            color="#3388ff" 
            weight={5} 
            opacity={0.7} 
          />
          
          {/* Ajustamos el mapa para mostrar todos los puntos */}
          <FitBounds waypoints={coordinates} />
        </MapContainer>
      )}
    </div>
  )
} 