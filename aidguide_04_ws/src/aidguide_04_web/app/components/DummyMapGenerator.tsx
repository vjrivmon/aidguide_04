"use client"

import { useEffect, useRef } from 'react'

interface DummyMapGeneratorProps {
  width: number
  height: number
  onMapGenerated: (dataUrl: string) => void
}

// Este componente genera un mapa de ocupación de cuadrícula simple para demostrar la integración
export default function DummyMapGenerator({ width, height, onMapGenerated }: DummyMapGeneratorProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  
  useEffect(() => {
    // Crear un canvas fuera del DOM para generar el mapa
    const canvas = document.createElement('canvas')
    canvas.width = width
    canvas.height = height
    canvasRef.current = canvas
    
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    
    // Llenar el canvas con color gris (desconocido)
    ctx.fillStyle = 'rgb(205, 205, 205)'
    ctx.fillRect(0, 0, width, height)
    
    // Dibujar un área libre (blanco) en el centro
    ctx.fillStyle = 'white'
    ctx.fillRect(width * 0.2, height * 0.2, width * 0.6, height * 0.6)
    
    // Dibujar algunos obstáculos (negro)
    ctx.fillStyle = 'black'
    
    // Paredes exteriores
    ctx.fillRect(width * 0.18, height * 0.18, width * 0.64, height * 0.04) // Superior
    ctx.fillRect(width * 0.18, height * 0.78, width * 0.64, height * 0.04) // Inferior
    ctx.fillRect(width * 0.18, height * 0.18, width * 0.04, height * 0.64) // Izquierda
    ctx.fillRect(width * 0.78, height * 0.18, width * 0.04, height * 0.64) // Derecha
    
    // Algunos obstáculos interiores
    ctx.fillRect(width * 0.35, height * 0.4, width * 0.1, height * 0.2) // Obstáculo central izquierdo
    ctx.fillRect(width * 0.55, height * 0.4, width * 0.1, height * 0.2) // Obstáculo central derecho
    
    // Convertir a URL de datos
    const dataUrl = canvas.toDataURL('image/png')
    onMapGenerated(dataUrl)
    
  }, [width, height, onMapGenerated])
  
  // Este componente no renderiza nada visible
  return null
} 