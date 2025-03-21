"use client"

import { useEffect, useState, useRef } from "react"
import { Canvas, useFrame } from "@react-three/fiber"
import { OrbitControls, useGLTF, Stage } from "@react-three/drei"
import { Suspense } from "react"
import * as THREE from 'three'

// Pre-cargar el modelo
useGLTF.preload('/robot_producto/source/ROBOT_ANIM_C.glb')

const INITIAL_POSITION = [0, -1.9, 0] as const

// Secuencia de animaciones con sus configuraciones
const ANIMATION_SEQUENCE = [
  { name: 'Saludo', duration: 3 },
  { name: 'Idle', duration: 2 },
  { name: 'Hablando', duration: 3 },
  { name: 'Idle1', duration: 2 },
  { name: 'Salto_Giro', duration: 2 },
  { name: 'Idle2', duration: 2 },
  { name: 'Despedida', duration: 3 }
]

function Model() {
  const modelRef = useRef<THREE.Group>(null)
  const [mixer, setMixer] = useState<THREE.AnimationMixer | null>(null)
  const [currentAnimationIndex, setCurrentAnimationIndex] = useState(0)

  // Cargar el modelo
  const { scene, animations } = useGLTF('/robot_producto/source/ROBOT_ANIM_C.glb', true)

  useEffect(() => {
    if (scene) {
      // Asegurar que los materiales se carguen correctamente
      scene.traverse((object) => {
        if (object instanceof THREE.Mesh) {
          object.frustumCulled = false // Evita que partes del modelo desaparezcan
          if (object.material) {
            object.material.needsUpdate = true
            object.material.side = THREE.DoubleSide // Renderiza ambos lados de las caras
          }
        }
      })

      if (animations.length > 0) {
        console.log("Todas las animaciones disponibles:", animations.map(a => ({
          nombre: a.name,
          duración: a.duration.toFixed(2)
        })))
        
        const newMixer = new THREE.AnimationMixer(scene)
        setMixer(newMixer)
      }
    }
  }, [scene, animations])

  useEffect(() => {
    if (!mixer || animations.length === 0) return

    // Detener todas las animaciones actuales
    mixer.stopAllAction()

    const animationConfig = ANIMATION_SEQUENCE[currentAnimationIndex]
    const animation = animations.find(anim => anim.name === animationConfig.name)

    if (animation) {
      console.log(`Iniciando animación: ${animationConfig.name}`)
      const action = mixer.clipAction(animation)
      action.reset()
      action.setLoop(THREE.LoopOnce, 1)
      action.clampWhenFinished = true
      action.play()

      // Programar la siguiente animación
      const duration = animation.duration * 1000 // Convertir a milisegundos
      setTimeout(() => {
        const nextIndex = (currentAnimationIndex + 1) % ANIMATION_SEQUENCE.length
        setCurrentAnimationIndex(nextIndex)
      }, duration)
    }

    return () => {
      mixer.stopAllAction()
    }
  }, [currentAnimationIndex, mixer, animations])

  // Actualizar el mixer
  useFrame((state, delta) => {
    mixer?.update(delta)
  })

  return (
    <group ref={modelRef}>
      <primitive 
        object={scene} 
        scale={5}
        position={INITIAL_POSITION}
        rotation={[0, 0, 0]}
        dispose={null}
        frustumCulled={false}
      />
    </group>
  )
}

export default function RobotProductoModel() {
  const [mounted, setMounted] = useState(false)

  useEffect(() => {
    setMounted(true)
  }, [])

  if (!mounted) return null

  return (
    <div className="w-full h-[500px] rounded-lg overflow-hidden">
      <Canvas
        camera={{ 
          position: [0, 0, 5],
          fov: 50,
          near: 0.1,
          far: 1000
        }}
        style={{ background: "transparent" }}
        shadows={false}
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
          preset="rembrandt"
          shadows={false}
        >
          <ambientLight intensity={1} />
          <spotLight 
            position={[10, 10, 10]} 
            angle={0.3} 
            penumbra={1} 
            intensity={2}
            castShadow={false}
          />
          <spotLight 
            position={[-10, -10, -10]} 
            angle={0.3} 
            penumbra={1} 
            intensity={1}
            castShadow={false}
          />
          <pointLight position={[0, 5, 0]} intensity={1} />
          <Suspense fallback={null}>
            <Model />
          </Suspense>
        </Stage>
        <OrbitControls 
          enableZoom={false}
          enableRotate={false}
          enablePan={false}
          autoRotate={false}
          target={[0, -2, 0]}
          maxPolarAngle={Math.PI / 2}
          minPolarAngle={0}
        />
      </Canvas>
    </div>
  )
} 