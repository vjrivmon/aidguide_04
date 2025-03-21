"use client"

import { useEffect, useState, useRef } from "react"
import { Canvas, useFrame } from "@react-three/fiber"
import { OrbitControls, useGLTF, Stage } from "@react-three/drei"
import { Suspense } from "react"
import * as THREE from 'three'

// Pre-cargar el modelo
useGLTF.preload('/robot_producto/source/ROBOT_ANIM_C.glb')

function Model() {
  const modelRef = useRef<THREE.Group>(null)
  const [currentAnimation, setCurrentAnimation] = useState(0)
  const [mixer, setMixer] = useState<THREE.AnimationMixer | null>(null)
  const [animations, setAnimations] = useState<THREE.AnimationClip[]>([])

  // Cargar el modelo
  const { scene, animations: loadedAnimations } = useGLTF('/robot_producto/source/ROBOT_ANIM_C.glb', true)

  // Orden de las animaciones
  const animationSequence = ['Saludo', 'Idle', 'Hablando', 'Idle1', 'Salto_Giro', 'Idle2', 'Despedida']

  useEffect(() => {
    if (scene && loadedAnimations.length > 0) {
      console.log("Animaciones disponibles:", loadedAnimations.map(a => a.name))
      const newMixer = new THREE.AnimationMixer(scene)
      setMixer(newMixer)
      setAnimations(loadedAnimations)

      // Iniciar con la primera animación
      playAnimation(0)
    }

    // Cleanup
    return () => {
      if (mixer) {
        mixer.stopAllAction()
        mixer.uncacheRoot(scene)
      }
    }
  }, [scene, loadedAnimations])

  // Usar useFrame para actualizar el mixer
  useFrame((state, delta) => {
    if (mixer) {
      mixer.update(delta)
    }
  })

  const playAnimation = (index: number) => {
    if (!mixer || animations.length === 0) return

    // Detener todas las animaciones actuales
    mixer.stopAllAction()

    // Encontrar la animación por nombre exacto
    const animationName = animationSequence[index]
    const animation = animations.find(anim => anim.name === animationName)

    console.log("Intentando reproducir:", animationName)
    console.log("Animación encontrada:", animation?.name)

    if (animation) {
      const action = mixer.clipAction(animation)
      action.reset()
      action.clampWhenFinished = true
      action.loop = THREE.LoopOnce
      
      // Configurar el callback para cuando termine la animación
      const onFinished = () => {
        // Pasar a la siguiente animación
        const nextIndex = (index + 1) % animationSequence.length
        setCurrentAnimation(nextIndex)
        
        // Remover el listener actual para evitar duplicados
        action.getMixer().removeEventListener('finished', onFinished)
        
        // Reproducir la siguiente animación después de un pequeño delay
        setTimeout(() => playAnimation(nextIndex), 100)
      }
      
      // Agregar el listener
      action.getMixer().addEventListener('finished', onFinished)
      
      // Reproducir la animación
      action.play()
    } else {
      // Si no se encuentra la animación, pasar a la siguiente
      const nextIndex = (index + 1) % animationSequence.length
      setCurrentAnimation(nextIndex)
      setTimeout(() => playAnimation(nextIndex), 100)
    }
  }

  return (
    <group ref={modelRef}>
      <primitive 
        object={scene} 
        scale={6}
        position={[0, -2, 0]}
        rotation={[0, Math.PI, 0]}
        dispose={null}
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
        camera={{ position: [0, 1, 5], fov: 45 }}
        style={{ background: "transparent" }}
      >
        <Stage
          intensity={0.5}
          environment="city"
          adjustCamera={false}
          preset="rembrandt"
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
            <Model />
          </Suspense>
        </Stage>
        <OrbitControls 
          enableZoom={true} 
          autoRotate={false}
          target={[0, 0, 0]}
          maxPolarAngle={Math.PI / 2}
          minPolarAngle={0}
        />
      </Canvas>
    </div>
  )
} 