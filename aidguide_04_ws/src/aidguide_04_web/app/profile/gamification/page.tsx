"use client"

import { useState, useEffect } from "react"
import Image from "next/image"
import Link from "next/link"
import { Trophy, Target, Gift, Award, Footprints, MapPin, Bus, Users, ArrowLeft, Clock, Star, Shield, Sparkles, Zap } from "lucide-react"
import { useAuth } from "@/context/auth-context"
import dynamic from 'next/dynamic'

// Importación dinámica para evitar errores de SSR
const ReactConfetti = dynamic(() => import('react-confetti'), {
  ssr: false
})

export default function GamificationPage() {
  const { user } = useAuth()
  const [activeTab, setActiveTab] = useState("challenges")
  const [showConfetti, setShowConfetti] = useState(false)
  const [confettiOpacity, setConfettiOpacity] = useState(1)
  const [windowSize, setWindowSize] = useState({ width: 0, height: 0 })
  const [claimedRewards, setClaimedRewards] = useState<number[]>([])
  
  // Efecto para obtener el tamaño de la ventana para el confeti
  useEffect(() => {
    const handleResize = () => {
      setWindowSize({
        width: window.innerWidth,
        height: window.innerHeight,
      })
    }
    
    // Establecer tamaño inicial
    handleResize()
    
    window.addEventListener('resize', handleResize)
    return () => window.removeEventListener('resize', handleResize)
  }, [])
  
  // Datos simulados del usuario
  const [userData, setUserData] = useState({
    name: "María García",
    totalPoints: 1250,
    level: 3,
    completedChallenges: 8,
    availableDiscounts: 2,
    levelProgress: 65, // Porcentaje hacia el siguiente nivel
    pointsToNextLevel: 350,
  })

  // Retos disponibles
  const challenges = [
    {
      id: 1,
      title: "Caminante diario",
      description: "Completa 15.000 pasos en un día",
      points: 150,
      progress: 65,
      icon: <Footprints size={24} className="text-button" />,
      category: "daily",
    },
    {
      id: 2,
      title: "Explorador urbano",
      description: "Completa 3 rutas diferentes en una semana",
      points: 200,
      progress: 33,
      icon: <MapPin size={24} className="text-button" />,
      category: "weekly",
    },
    {
      id: 3,
      title: "Amante de la tecnología",
      description: "Usa la aplicación durante 7 días consecutivos",
      points: 100,
      progress: 85,
      icon: <Clock size={24} className="text-button" />,
      category: "streak",
    },
    {
      id: 4,
      title: "Viajero frecuente",
      description: "Utiliza el transporte público 5 veces en una semana",
      points: 175,
      progress: 20,
      icon: <Bus size={24} className="text-button" />,
      category: "weekly",
    },
    {
      id: 5,
      title: "Socialización",
      description: "Visita 3 lugares concurridos esta semana",
      points: 125,
      progress: 0,
      icon: <Users size={24} className="text-button" />,
      category: "weekly",
    },
  ]

  // Recompensas disponibles
  const rewards = [
    {
      id: 1,
      title: "10% de descuento en transporte público",
      description: "Vale válido para un viaje en cualquier transporte público",
      points: 500,
      provider: "Metro Valencia",
      icon: <Bus size={24} className="text-button" />,
      expiry: "31/12/2025",
    },
    {
      id: 2,
      title: "5€ en tu cafetería favorita",
      description: "Descuento aplicable en cualquier compra superior a 10€",
      points: 300,
      provider: "Café Central",
      icon: <Gift size={24} className="text-button" />,
      expiry: "30/06/2025",
    },
    {
      id: 3,
      title: "Entrada gratuita al museo",
      description: "Visita cualquier museo de la ciudad sin coste",
      points: 750,
      provider: "Museos Municipales",
      icon: <Award size={24} className="text-button" />,
      expiry: "31/12/2025",
    },
    {
      id: 4,
      title: "Descuento de 15% en tiendas deportivas",
      description: "Válido para compras de más de 50€",
      points: 600,
      provider: "Decathlon",
      icon: <Footprints size={24} className="text-button" />,
      expiry: "30/09/2025",
    },
  ]

  // Logros desbloqueados
  const achievements = [
    {
      id: 1,
      title: "Primeros pasos",
      description: "Completaste tu primera ruta con AidGuide",
      icon: <Footprints size={24} className="text-white" />,
      date: "15/03/2024",
      badgeColor: "bg-green-500",
    },
    {
      id: 2,
      title: "Explorador novato",
      description: "Has visitado 5 lugares diferentes",
      icon: <MapPin size={24} className="text-white" />,
      date: "18/03/2024",
      badgeColor: "bg-blue-500",
    },
    {
      id: 3,
      title: "Viajero social",
      description: "Has utilizado el transporte público 10 veces",
      icon: <Bus size={24} className="text-white" />,
      date: "25/03/2024",
      badgeColor: "bg-purple-500",
    },
  ]

  // Función para canjear recompensa
  const claimReward = (reward: any) => {
    if (userData.totalPoints >= reward.points) {
      // Mostrar confeti con opacidad completa
      setConfettiOpacity(1)
      setShowConfetti(true)
      
      // Actualizar puntos del usuario
      setUserData({
        ...userData,
        totalPoints: userData.totalPoints - reward.points
      })
      
      // Añadir a la lista de recompensas canjeadas
      setClaimedRewards([...claimedRewards, reward.id])
      
      // Iniciar el desvanecimiento después de 2 segundos
      setTimeout(() => {
        // Desvanecimiento gradual durante 2 segundos
        const fadeInterval = setInterval(() => {
          setConfettiOpacity((prevOpacity) => {
            const newOpacity = prevOpacity - 0.05;
            if (newOpacity <= 0) {
              clearInterval(fadeInterval);
              setShowConfetti(false);
              return 0;
            }
            return newOpacity;
          });
        }, 100);
      }, 2000);
    }
  }

  return (
    <div className="container-custom py-10">
      {/* Componente Confetti con transición de opacidad */}
      {showConfetti && (
        <div className="fixed inset-0 z-50 pointer-events-none transition-opacity duration-300 ease-out"
             style={{ opacity: confettiOpacity }}>
          <ReactConfetti
            width={windowSize.width}
            height={windowSize.height}
            recycle={true}
            numberOfPieces={500}
            gravity={0.15}
          />
        </div>
      )}
      {/* Cabecera de la página */}
      <div className="flex items-center justify-between mb-8">
        <div className="flex items-center">
          <Link href="/profile" className="mr-4 p-2 hover:bg-gray-100 rounded-full transition-colors">
            <ArrowLeft size={24} className="text-button" />
          </Link>
          <div>
            <h1 className="text-3xl md:text-4xl font-bold">Recompensas</h1>
            <p className="text-gray-600">Completa retos y canjea puntos por recompensas</p>
          </div>
        </div>
      </div>
      
      {/* Resumen de nivel y puntos */}
      <div className="bg-gradient-to-r from-button to-blue-600 rounded-lg p-6 text-white mb-8">
        <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
          <div className="col-span-2">
            <div className="flex items-center mb-4">
              <div className="bg-white/20 p-3 rounded-full mr-4">
                <Trophy size={28} />
              </div>
              <div>
                <h2 className="text-2xl font-bold">Nivel {userData.level}</h2>
                <p className="opacity-80">¡Sigue completando retos para subir de nivel!</p>
              </div>
            </div>
            
            {/* Barra de progreso para el siguiente nivel */}
            <div>
              <div className="flex justify-between text-sm mb-2">
                <span>Nivel {userData.level}</span>
                <span>Nivel {userData.level + 1}</span>
              </div>
              <div className="w-full bg-white/30 rounded-full h-4">
                <div 
                  className="bg-white h-4 rounded-full transition-all duration-500 ease-in-out" 
                  style={{ width: `${userData.levelProgress}%` }}
                ></div>
              </div>
              <p className="text-sm mt-2">{userData.pointsToNextLevel} puntos más para el siguiente nivel</p>
            </div>
          </div>
          
          <div className="bg-white/10 rounded-lg p-4 flex flex-col items-center justify-center text-center">
            <div className="bg-white/20 p-4 rounded-full mb-3">
              <Sparkles size={32} />
            </div>
            <p className="text-lg">Total de puntos</p>
            <p className="text-3xl font-bold">{userData.totalPoints}</p>
            <p className="text-sm opacity-80 mt-2">Has completado {userData.completedChallenges} retos</p>
          </div>
        </div>
      </div>
      
      {/* Navegación entre pestañas */}
      <div className="bg-white rounded-lg shadow-md mb-8">
        <div className="flex border-b">
          <button
            onClick={() => setActiveTab("challenges")}
            className={`flex-1 py-4 text-center font-medium transition-colors ${
              activeTab === "challenges"
                ? "text-button border-b-2 border-button"
                : "text-gray-500 hover:text-button"
            }`}
          >
            <div className="flex items-center justify-center">
              <Target size={18} className="mr-2" />
              Retos
            </div>
          </button>
          <button
            onClick={() => setActiveTab("rewards")}
            className={`flex-1 py-4 text-center font-medium transition-colors ${
              activeTab === "rewards"
                ? "text-button border-b-2 border-button"
                : "text-gray-500 hover:text-button"
            }`}
          >
            <div className="flex items-center justify-center">
              <Gift size={18} className="mr-2" />
              Recompensas
            </div>
          </button>
          <button
            onClick={() => setActiveTab("achievements")}
            className={`flex-1 py-4 text-center font-medium transition-colors ${
              activeTab === "achievements"
                ? "text-button border-b-2 border-button"
                : "text-gray-500 hover:text-button"
            }`}
          >
            <div className="flex items-center justify-center">
              <Award size={18} className="mr-2" />
              Logros
            </div>
          </button>
        </div>
      </div>
      
      {/* Contenido de la tab activa */}
      <div className="bg-white rounded-lg shadow-md p-6">
        {/* Tab de Retos */}
        {activeTab === "challenges" && (
          <div>
            <div className="mb-6">
              <h2 className="text-2xl font-bold text-button mb-1">Tus Retos</h2>
              <p className="text-gray-600">Completa estos retos para ganar puntos y subir de nivel</p>
            </div>
            
            {/* Filtros de retos */}
            <div className="flex gap-2 mb-6 overflow-x-auto pb-2">
              <button className="px-4 py-2 bg-button text-white rounded-full text-sm font-medium">
                Todos
              </button>
              <button className="px-4 py-2 bg-gray-100 text-gray-700 rounded-full text-sm font-medium hover:bg-gray-200">
                Diarios
              </button>
              <button className="px-4 py-2 bg-gray-100 text-gray-700 rounded-full text-sm font-medium hover:bg-gray-200">
                Semanales
              </button>
              <button className="px-4 py-2 bg-gray-100 text-gray-700 rounded-full text-sm font-medium hover:bg-gray-200">
                Racha
              </button>
            </div>
            
            {/* Lista de retos */}
            <div className="space-y-4">
              {challenges.map((challenge) => (
                <div key={challenge.id} className="bg-gray-50 p-4 rounded-lg border border-gray-100 hover:shadow-md transition-shadow">
                  <div className="flex items-start gap-3">
                    <div className="p-3 bg-gray-100 rounded-full">
                      {challenge.icon}
                    </div>
                    <div className="flex-1">
                      <div className="flex justify-between items-center">
                        <h3 className="font-medium text-lg">{challenge.title}</h3>
                        <div className="flex items-center bg-button/10 py-1 px-3 rounded-full">
                          <Star size={16} className="text-button mr-1" />
                          <span className="font-bold text-button">{challenge.points} pts</span>
                        </div>
                      </div>
                      <p className="text-gray-600 mb-3">{challenge.description}</p>
                      <div className="w-full bg-gray-200 rounded-full h-2.5 mb-1">
                        <div 
                          className="bg-button h-2.5 rounded-full transition-all duration-500 ease-in-out" 
                          style={{ width: `${challenge.progress}%` }}
                        ></div>
                      </div>
                      <div className="flex justify-between text-xs text-gray-500">
                        <span>Progreso: {challenge.progress}%</span>
                        <span>{Math.round(challenge.progress * 0.01 * challenge.points)} / {challenge.points} pts</span>
                      </div>
                    </div>
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}
        
        {/* Tab de Recompensas */}
        {activeTab === "rewards" && (
          <div>
            <div className="mb-6">
              <h2 className="text-2xl font-bold text-button mb-1">Recompensas Disponibles</h2>
              <p className="text-gray-600">Canjea tus puntos por estas fantásticas recompensas</p>
            </div>
            
            {/* Información de puntos */}
            <div className="bg-gray-50 p-4 rounded-lg mb-6 flex items-center justify-between">
              <div className="flex items-center">
                <Trophy size={20} className="text-button mr-2" />
                <span className="font-medium">Tus puntos disponibles:</span>
              </div>
              <span className="text-xl font-bold text-button">{userData.totalPoints} pts</span>
            </div>
            
            {/* Lista de recompensas */}
            <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
              {rewards.filter(reward => !claimedRewards.includes(reward.id)).map((reward) => (
                <div key={reward.id} className="bg-white border border-gray-100 rounded-lg overflow-hidden shadow-sm hover:shadow-md transition-shadow">
                  <div className="p-4">
                    <div className="flex items-start gap-3">
                      <div className="p-3 bg-gray-100 rounded-full">
                        {reward.icon}
                      </div>
                      <div className="flex-1">
                        <div className="flex justify-between">
                          <h3 className="font-medium">{reward.title}</h3>
                          <span className="font-bold text-button">{reward.points} pts</span>
                        </div>
                        <p className="text-sm text-gray-600 mb-1">{reward.description}</p>
                        <div className="flex justify-between items-center text-xs text-gray-500">
                          <span>Proveedor: {reward.provider}</span>
                          <span>Válido hasta: {reward.expiry}</span>
                        </div>
                      </div>
                    </div>
                  </div>
                  <div className="border-t p-3 bg-gray-50">
                    <button 
                      className={`w-full py-2 rounded-lg text-sm font-medium transition-colors ${
                        userData.totalPoints >= reward.points
                          ? "bg-button text-white hover:bg-opacity-90"
                          : "bg-gray-200 text-gray-500 cursor-not-allowed"
                      }`}
                      disabled={userData.totalPoints < reward.points}
                      onClick={() => claimReward(reward)}
                    >
                      {userData.totalPoints >= reward.points 
                        ? "Canjear recompensa" 
                        : `Necesitas ${reward.points - userData.totalPoints} pts más`}
                    </button>
                  </div>
                </div>
              ))}
              
              {/* Mensaje cuando no hay recompensas disponibles */}
              {rewards.length > 0 && claimedRewards.length === rewards.length && (
                <div className="col-span-full bg-gray-50 p-6 rounded-lg text-center">
                  <div className="flex justify-center mb-4">
                    <Trophy size={40} className="text-button" />
                  </div>
                  <h3 className="text-xl font-medium mb-2">¡Has canjeado todas las recompensas disponibles!</h3>
                  <p className="text-gray-600">Vuelve pronto para descubrir nuevas recompensas.</p>
                </div>
              )}
            </div>
          </div>
        )}
        
        {/* Tab de Logros */}
        {activeTab === "achievements" && (
          <div>
            <div className="mb-6">
              <h2 className="text-2xl font-bold text-button mb-1">Tus Logros</h2>
              <p className="text-gray-600">Un registro de tus hitos conseguidos con AidGuide</p>
            </div>
            
            {/* Estadísticas generales */}
            <div className="grid grid-cols-1 md:grid-cols-3 gap-4 mb-6">
              <div className="bg-gray-50 p-4 rounded-lg flex items-center">
                <div className="p-3 bg-button rounded-full mr-3">
                  <Trophy size={20} className="text-white" />
                </div>
                <div>
                  <p className="text-sm text-gray-500">Logros desbloqueados</p>
                  <p className="text-xl font-bold">{achievements.length}</p>
                </div>
              </div>
              <div className="bg-gray-50 p-4 rounded-lg flex items-center">
                <div className="p-3 bg-blue-500 rounded-full mr-3">
                  <Zap size={20} className="text-white" />
                </div>
                <div>
                  <p className="text-sm text-gray-500">Puntos ganados</p>
                  <p className="text-xl font-bold">{userData.totalPoints}</p>
                </div>
              </div>
              <div className="bg-gray-50 p-4 rounded-lg flex items-center">
                <div className="p-3 bg-purple-500 rounded-full mr-3">
                  <Shield size={20} className="text-white" />
                </div>
                <div>
                  <p className="text-sm text-gray-500">Nivel actual</p>
                  <p className="text-xl font-bold">{userData.level}</p>
                </div>
              </div>
            </div>
            
            {/* Lista de logros */}
            <div className="space-y-4">
              {achievements.map((achievement) => (
                <div key={achievement.id} className="bg-gray-50 p-4 rounded-lg border border-gray-100 flex items-start gap-4">
                  <div className={`p-3 ${achievement.badgeColor} rounded-full`}>
                    {achievement.icon}
                  </div>
                  <div className="flex-1">
                    <div className="flex justify-between items-center">
                      <h3 className="font-medium">{achievement.title}</h3>
                      <span className="text-sm text-gray-500">{achievement.date}</span>
                    </div>
                    <p className="text-gray-600">{achievement.description}</p>
                  </div>
                </div>
              ))}
            </div>
          </div>
        )}
      </div>
    </div>
  )
} 