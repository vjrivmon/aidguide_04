"use client"

import { useEffect, useState, useRef } from "react"
import { Canvas, useFrame } from "@react-three/fiber"
import { OrbitControls, useGLTF, Box, SpotLight, Stage } from "@react-three/drei"
import { Suspense } from "react"
import * as THREE from 'three'

function Model({ url }: { url: string }) {
  const [error, setError] = useState<string | null>(null)
  const modelRef = useRef<THREE.Group>(null)

  // Cargar el modelo
  const { scene, animations } = useGLTF(url)

  // Rotación automática
  useFrame((state, delta) => {
    if (modelRef.current) {
      modelRef.current.rotation.y += delta * 0.00001
    }
  })

  if (!scene) {
    console.error("No se pudo cargar la escena del modelo")
    return (
      <Box args={[1, 1, 1]}>
        <meshStandardMaterial color="red" />
      </Box>
    )
  }

  return (
    <group ref={modelRef}>
      <primitive 
        object={scene} 
        scale={0.007}
        position={[0, 0, 0]}
      />
    </group>
  )
}

export default function RobotModel() {
  const [mounted, setMounted] = useState(false)

  useEffect(() => {
    setMounted(true)
    // Precargar el modelo
    useGLTF.preload('/scene.gltf')
  }, [])

  if (!mounted) return null

  return (
    <div className="w-full h-[500px] rounded-lg overflow-hidden">
      <Canvas
        camera={{ position: [0, 1, 3], fov: 50 }}
        style={{ background: "transparent" }}
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
          <Suspense fallback={null}>
            <Model url="/scene.gltf" />
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

