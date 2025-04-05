"use client"

import { ReactNode, createContext, useContext, useEffect, useState } from "react"
import ROSLIB from 'roslib'

// Interfaz para las notificaciones
interface Notification {
  id: string;
  message: string;
  type: 'weather' | 'battery' | 'system' | 'maintenance';
  timestamp: Date;
  read: boolean;
}

interface RobotContextType {
  batteryPercentage: number
  batteryStatus: string // 'charging' | 'discharging'
  estimatedTimeRemaining: string
  isConnected: boolean
  notifications: Notification[]
  markNotificationAsRead: (id: string) => void
  reconnect: () => void
}

const defaultContext: RobotContextType = {
  batteryPercentage: 75,
  batteryStatus: 'discharging',
  estimatedTimeRemaining: '4 horas',
  isConnected: false,
  notifications: [],
  markNotificationAsRead: () => {},
  reconnect: () => {}
}

const RobotContext = createContext<RobotContextType>(defaultContext)

export function RobotProvider({ children }: { children: ReactNode }) {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null)
  const [isConnected, setIsConnected] = useState(false)
  const [batteryPercentage, setBatteryPercentage] = useState(75)
  const [batteryStatus, setBatteryStatus] = useState('discharging')
  const [estimatedTimeRemaining, setEstimatedTimeRemaining] = useState('4 horas')
  const [notifications, setNotifications] = useState<Notification[]>([])

  // Marcar una notificación como leída
  const markNotificationAsRead = (id: string) => {
    setNotifications(prevNotifications => 
      prevNotifications.map(notification => 
        notification.id === id ? { ...notification, read: true } : notification
      )
    )
  }

  // Calcular el tiempo estimado basado en el porcentaje de batería y estado
  const calculateEstimatedTime = (percentage: number, status: string): string => {
    if (status === 'charging') {
      // Tiempo estimado para carga completa
      const minutesToFull = Math.round((100 - percentage) * 5) // Estimación simple: 5 minutos por cada 1%
      if (minutesToFull < 60) {
        return `${minutesToFull} minutos para carga completa`
      } else {
        const hours = Math.floor(minutesToFull / 60)
        const minutes = minutesToFull % 60
        return `${hours} hora${hours !== 1 ? 's' : ''} ${minutes > 0 ? `${minutes} min` : ''} para carga completa`
      }
    } else {
      // Tiempo estimado restante de batería
      const minutesRemaining = Math.round(percentage * 5) // Estimación simple: 5 minutos por cada 1%
      if (minutesRemaining < 60) {
        return `${minutesRemaining} minutos`
      } else {
        const hours = Math.floor(minutesRemaining / 60)
        return `${hours} hora${hours !== 1 ? 's' : ''}`
      }
    }
  }

  const connectToROS = () => {
    try {
      const rosInstance = new ROSLIB.Ros({
        url: 'ws://localhost:9090' // Asume que el puente websocket está en localhost:9090
      })

      rosInstance.on('connection', () => {
        console.log('Conectado al servidor de ROS')
        setIsConnected(true)
      })

      rosInstance.on('error', (error) => {
        console.error('Error conectando a ROS:', error)
        setIsConnected(false)
      })

      rosInstance.on('close', () => {
        console.log('Conexión a ROS cerrada')
        setIsConnected(false)
      })

      setRos(rosInstance)
    } catch (error) {
      console.error('Error iniciando conexión ROS:', error)
      setIsConnected(false)
    }
  }

  const reconnect = () => {
    if (ros) {
      ros.close()
    }
    connectToROS()
  }

  useEffect(() => {
    connectToROS()

    return () => {
      if (ros) {
        ros.close()
      }
    }
  }, [])

  useEffect(() => {
    if (!ros || !isConnected) return

    // Suscribirse al tema de batería
    const batteryTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/battery_status',
      messageType: 'sensor_msgs/BatteryState'
    })

    batteryTopic.subscribe((message: any) => {
      // Actualizar estado de la batería
      const percentage = Math.round(message.percentage)
      setBatteryPercentage(percentage)
      
      // Determinar si está cargando o descargando
      const status = message.power_supply_status === 1 ? 'charging' : 'discharging'
      setBatteryStatus(status)
      
      // Actualizar tiempo estimado
      setEstimatedTimeRemaining(calculateEstimatedTime(percentage, status))
    })

    // Suscribirse al tema de alertas del clima
    const weatherTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/weather_alert',
      messageType: 'std_msgs/String'
    })

    weatherTopic.subscribe((message: any) => {
      // Crear una nueva notificación de clima
      const weatherNotification: Notification = {
        id: Date.now().toString(),
        message: message.data,
        type: 'weather',
        timestamp: new Date(),
        read: false
      }
      
      // Añadir la notificación al principio de la lista (más reciente primero)
      setNotifications(prev => [weatherNotification, ...prev])
    })

    return () => {
      batteryTopic.unsubscribe()
      weatherTopic.unsubscribe()
    }
  }, [ros, isConnected])

  return (
    <RobotContext.Provider
      value={{
        batteryPercentage,
        batteryStatus,
        estimatedTimeRemaining,
        isConnected,
        notifications,
        markNotificationAsRead,
        reconnect
      }}
    >
      {children}
    </RobotContext.Provider>
  )
}

export const useRobot = () => useContext(RobotContext) 