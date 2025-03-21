"use client"

import { useRef, useEffect } from "react"

export default function SimpleRobotModel() {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    if (!canvasRef.current) return

    const canvas = canvasRef.current
    const ctx = canvas.getContext("2d")
    if (!ctx) return

    // Configurar el canvas
    const dpr = window.devicePixelRatio || 1
    const rect = canvas.getBoundingClientRect()
    canvas.width = rect.width * dpr
    canvas.height = rect.height * dpr
    ctx.scale(dpr, dpr)

    // Dibujar un robot perro simple
    const drawRobotDog = () => {
      if (!ctx) return

      // Limpiar canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height)

      // Configurar estilo
      ctx.fillStyle = "#2867B2"
      ctx.strokeStyle = "#2867B2"
      ctx.lineWidth = 3

      const centerX = rect.width / 2
      const centerY = rect.height / 2
      const size = Math.min(rect.width, rect.height) * 0.4

      // Cuerpo
      ctx.beginPath()
      ctx.ellipse(centerX, centerY, size * 0.6, size * 0.3, 0, 0, Math.PI * 2)
      ctx.stroke()

      // Cabeza
      ctx.beginPath()
      ctx.arc(centerX - size * 0.5, centerY, size * 0.25, 0, Math.PI * 2)
      ctx.stroke()

      // Ojos
      ctx.beginPath()
      ctx.arc(centerX - size * 0.6, centerY - size * 0.05, size * 0.05, 0, Math.PI * 2)
      ctx.fill()

      // Orejas
      ctx.beginPath()
      ctx.moveTo(centerX - size * 0.65, centerY - size * 0.15)
      ctx.lineTo(centerX - size * 0.8, centerY - size * 0.4)
      ctx.lineTo(centerX - size * 0.5, centerY - size * 0.2)
      ctx.closePath()
      ctx.fill()

      // Patas
      const drawLeg = (offsetX: number, offsetY: number) => {
        ctx.beginPath()
        ctx.moveTo(centerX + offsetX, centerY + offsetY)
        ctx.lineTo(centerX + offsetX, centerY + offsetY + size * 0.4)
        ctx.stroke()
      }

      // Patas delanteras
      drawLeg(-size * 0.4, size * 0.2)
      drawLeg(-size * 0.2, size * 0.2)

      // Patas traseras
      drawLeg(size * 0.2, size * 0.2)
      drawLeg(size * 0.4, size * 0.2)

      // Cola
      ctx.beginPath()
      ctx.moveTo(centerX + size * 0.6, centerY)
      ctx.quadraticCurveTo(centerX + size * 0.8, centerY - size * 0.3, centerX + size * 0.7, centerY - size * 0.4)
      ctx.stroke()
    }

    // Dibujar inicialmente
    drawRobotDog()

    // Animar
    let angle = 0
    const animate = () => {
      if (!ctx) return
      angle += 0.05

      // Mover ligeramente el robot
      ctx.save()
      ctx.translate(rect.width / 2, rect.height / 2)
      ctx.rotate(Math.sin(angle) * 0.05)
      ctx.translate(-rect.width / 2, -rect.height / 2)

      drawRobotDog()
      ctx.restore()

      requestAnimationFrame(animate)
    }

    const animationId = requestAnimationFrame(animate)

    return () => {
      cancelAnimationFrame(animationId)
    }
  }, [])

  return (
    <div className="w-full h-80 md:h-96 bg-gray-50 rounded-lg flex items-center justify-center">
      <canvas ref={canvasRef} className="w-full h-full" style={{ maxWidth: "100%", maxHeight: "100%" }} />
    </div>
  )
}

