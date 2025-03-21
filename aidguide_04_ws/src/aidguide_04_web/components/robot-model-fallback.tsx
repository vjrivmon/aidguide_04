"use client"

import { useRef, useEffect } from "react"

export default function RobotModelFallback() {
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

    // Dibujar un robot simple
    const drawRobot = () => {
      if (!ctx) return

      // Limpiar canvas
      ctx.clearRect(0, 0, canvas.width, canvas.height)

      // Configurar estilo
      ctx.fillStyle = "#2867B2"
      ctx.strokeStyle = "#2867B2"
      ctx.lineWidth = 2

      // Dibujar cuerpo
      const centerX = rect.width / 2
      const centerY = rect.height / 2
      const size = Math.min(rect.width, rect.height) * 0.4

      // Cabeza
      ctx.beginPath()
      ctx.roundRect(centerX - size / 2, centerY - size / 2, size, size, 10)
      ctx.stroke()

      // Ojos
      ctx.beginPath()
      ctx.arc(centerX - size / 4, centerY - size / 6, size / 10, 0, Math.PI * 2)
      ctx.fill()

      ctx.beginPath()
      ctx.arc(centerX + size / 4, centerY - size / 6, size / 10, 0, Math.PI * 2)
      ctx.fill()

      // Boca
      ctx.beginPath()
      ctx.arc(centerX, centerY + size / 6, size / 5, 0, Math.PI)
      ctx.stroke()

      // Orejas
      ctx.beginPath()
      ctx.moveTo(centerX - size / 2, centerY - size / 4)
      ctx.lineTo(centerX - size / 2 - size / 3, centerY - size / 2)
      ctx.lineTo(centerX - size / 2, centerY - size / 2)
      ctx.fill()

      ctx.beginPath()
      ctx.moveTo(centerX + size / 2, centerY - size / 4)
      ctx.lineTo(centerX + size / 2 + size / 3, centerY - size / 2)
      ctx.lineTo(centerX + size / 2, centerY - size / 2)
      ctx.fill()
    }

    // Dibujar inicialmente
    drawRobot()

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

      drawRobot()
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

