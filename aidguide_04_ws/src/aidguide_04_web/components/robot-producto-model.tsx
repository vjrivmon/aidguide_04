"use client"

import { useEffect, useState, useRef } from "react"
import { Canvas, useFrame } from "@react-three/fiber"
import { OrbitControls, useGLTF, Stage } from "@react-three/drei"
import { Suspense } from "react"
import * as THREE from 'three'

// Pre-cargar el modelo
useGLTF.preload('/robot_producto/source/ROBOT_ANIM_C.glb')

const INITIAL_POSITION = [0, -1.9, 0] as const
function Model() {
  const modelRef = useRef<THREE.Group>(null)
  const [mixer, setMixer] = useState<THREE.AnimationMixer | null>(null)
  const currentActionRef = useRef<THREE.AnimationAction | null>(null)

  // Cargar el modelo
  const { scene, animations } = useGLTF('/robot_producto/source/ROBOT_ANIM_C.glb', true)

  useEffect(() => {
    // Asegurar que el modelo siempre comience en la posición inicial
    if (scene) {
      scene.position.set(INITIAL_POSITION[0], INITIAL_POSITION[1], INITIAL_POSITION[2])
    }

    if (scene && animations.length > 0) {
      console.log("Animaciones disponibles:", animations.map(a => a.name))
      const newMixer = new THREE.AnimationMixer(scene)
      setMixer(newMixer)

      // Encontrar la animación "Idle"
      const idleAnimation = animations.find(anim => anim.name === 'Idle')
      
      if (idleAnimation) {
        const action = newMixer.clipAction(idleAnimation)
        currentActionRef.current = action
        
        // Configurar la animación para bucle infinito
        action.reset()
        action.clampWhenFinished = false
        action.loop = THREE.LoopRepeat
        action.play()
      }
    }

    return () => {
      if (mixer) {
        mixer.stopAllAction()
        mixer.uncacheRoot(scene)
      }
      if (currentActionRef.current) {
        currentActionRef.current.stop()
      }
    }
  }, [scene, animations])

  // Usar useFrame para actualizar el mixer y mantener la posición
  useFrame((state, delta) => {
    if (mixer) {
      mixer.update(delta)
    }
    // Asegurar que el modelo se mantenga en la posición correcta
    if (scene) {
      scene.position.set(INITIAL_POSITION[0], INITIAL_POSITION[1], INITIAL_POSITION[2])
    }
  })

  return (
    <group ref={modelRef}>
      <primitive 
        object={scene} 
        scale={5}
        position={INITIAL_POSITION}
        rotation={[0, 0, 0]}
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
        camera={{ 
          position: [0, 0, 5],
          fov: 50,
          near: 0.1,
          far: 1000
        }}
        style={{ background: "transparent" }}
        shadows={false}
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