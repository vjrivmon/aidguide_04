"use client"

import { useEffect, useState, useRef } from "react"
import { Canvas, useFrame } from "@react-three/fiber"
import { OrbitControls, useGLTF, Box, SpotLight, Stage } from "@react-three/drei"
import { Suspense } from "react"
import * as THREE from 'three'

function Model({ url }: { url: string }) {
  const [error, setError] = useState<string | null>(null)
  const modelRef = useRef<THREE.Group>(null)

  // Cargar el modelo con manejo de errores
  const { scene, animations } = useGLTF(url, true)

  useEffect(() => {
    if (error) {
      console.error("Error al cargar el modelo:", error)
    }
  }, [error])

  // Rotación automática
  useFrame((state, delta) => {
    if (modelRef.current) {
      modelRef.current.rotation.y += delta * 0.5
    }
  })

  if (error) {
    return (
      <Box args={[1, 1, 1]}>
        <meshStandardMaterial color="red" />
      </Box>
    )
  }

  if (!scene) {
    return (
      <Box args={[1, 1, 1]}>
        <meshStandardMaterial color="blue" />
      </Box>
    )
  }

  return (
    <group ref={modelRef}>
      <primitive 
        object={scene} 
        scale={0.007}
        position={[0, 0, 0]}
        onError={(e) => setError(e.message)}
      />
    </group>
  )
}

export default function RobotModel() {
  const [mounted, setMounted] = useState(false)
  const [modelUrl, setModelUrl] = useState('/scene.gltf')

  useEffect(() => {
    setMounted(true)
    // Precargar el modelo
    useGLTF.preload(modelUrl)
  }, [modelUrl])

  if (!mounted) {
    return (
      <div className="w-full h-80 md:h-96 flex items-center justify-center bg-gradient-to-b from-blue-50 to-gray-100 rounded-33 shadow-lg">
        <div className="animate-pulse flex flex-col items-center">
          <div className="w-24 h-24 bg-blue-200 rounded-full mb-4"></div>
          <div className="h-4 w-32 bg-blue-200 rounded mb-2"></div>
          <div className="h-3 w-24 bg-blue-100 rounded"></div>
        </div>
      </div>
    )
  }

  return (
    <div className="w-full h-[500px] rounded-lg overflow-hidden">
      <Canvas
        camera={{ position: [0, 1, 3], fov: 50 }}
        style={{ background: "transparent" }}
        gl={{ 
          antialias: true,
          alpha: true,
          preserveDrawingBuffer: true
        }}
      >
        <Stage
          intensity={0.5}
          environment="city"
          adjustCamera={false}
        >
          <ambientLight intensity={0.8} />
          <spotLight 
            position={[10, 10, 10]} 
            angle={0.3} 
            penumbra={1} 
            intensity={1.5}
            castShadow
          />
          <spotLight 
            position={[-10, -10, -10]} 
            angle={0.3} 
            penumbra={1} 
            intensity={0.5}
            castShadow
          />
          <pointLight position={[0, 5, 0]} intensity={0.5} />
          <Suspense fallback={
            <Box args={[1, 1, 1]}>
              <meshStandardMaterial color="gray" />
            </Box>
          }>
            <Model url={modelUrl} />
          </Suspense>
        </Stage>
        <OrbitControls 
          enableZoom={true} 
          autoRotate={true}
          autoRotateSpeed={2.0}
          target={[0, 0, 0]}
        />
      </Canvas>
    </div>
  )
}

// Limpiar la caché cuando el componente se desmonte
useGLTF.preload('/scene.gltf')

