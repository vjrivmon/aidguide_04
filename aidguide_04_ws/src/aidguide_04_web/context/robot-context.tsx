"use client";

import { ReactNode, createContext, useContext, useEffect, useState } from "react";
import ROSLIB from "roslib";

// Interfaz para las notificaciones
interface Notification {
  id: string;
  message: string;
  type: "weather" | "battery" | "system" | "maintenance";
  timestamp: Date;
  read: boolean;
}

// Interfaz para la posición del robot
interface RobotPose {
  x: number;
  y: number;
}

// Interfaz para Topic de ROSLIB
interface ROSLIBTopic {
  ros: ROSLIB.Ros;
  name: string;
  messageType: string;
  subscribe: (callback: (message: any) => void) => void;
  unsubscribe: () => void;
}

// Interfaz actualizada del contexto
interface RobotContextType {
  batteryPercentage: number;
  batteryStatus: string; // 'charging' | 'discharging'
  estimatedTimeRemaining: string;
  isConnected: boolean;
  notifications: Notification[];
  robotPose: RobotPose; // Añadimos la posición del robot
  markNotificationAsRead: (id: string) => void;
  reconnect: () => void;
}

const defaultContext: RobotContextType = {
  batteryPercentage: 75,
  batteryStatus: "discharging",
  estimatedTimeRemaining: "4 horas",
  isConnected: false,
  notifications: [],
  robotPose: { x: 0, y: 0 }, // Valor inicial para robotPose
  markNotificationAsRead: () => {},
  reconnect: () => {},
};

const RobotContext = createContext<RobotContextType>(defaultContext);

export function RobotProvider({ children }: { children: ReactNode }) {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const [batteryPercentage, setBatteryPercentage] = useState(75);
  const [batteryStatus, setBatteryStatus] = useState("discharging");
  const [estimatedTimeRemaining, setEstimatedTimeRemaining] = useState("4 horas");
  const [notifications, setNotifications] = useState<Notification[]>([]);
  const [robotPose, setRobotPose] = useState<RobotPose>({ x: 0, y: 0 }); // Estado para la posición del robot

  // Marcar una notificación como leída
  const markNotificationAsRead = (id: string) => {
    setNotifications((prevNotifications) =>
      prevNotifications.map((notification) =>
        notification.id === id ? { ...notification, read: true } : notification
      )
    );
  };

  // Calcular el tiempo estimado basado en el porcentaje de batería y estado
  const calculateEstimatedTime = (percentage: number, status: string): string => {
    if (status === "charging") {
      const minutesToFull = Math.round((100 - percentage) * 5); // 5 min por 1%
      if (minutesToFull < 60) {
        return `${minutesToFull} minutos para carga completa`;
      } else {
        const hours = Math.floor(minutesToFull / 60);
        const minutes = minutesToFull % 60;
        return `${hours} hora${hours !== 1 ? "s" : ""} ${minutes > 0 ? `${minutes} min` : ""} para carga completa`;
      }
    } else {
      const minutesRemaining = Math.round(percentage * 5); // 5 min por 1%
      if (minutesRemaining < 60) {
        return `${minutesRemaining} minutos`;
      } else {
        const hours = Math.floor(minutesRemaining / 60);
        return `${hours} hora${hours !== 1 ? "s" : ""}`;
      }
    }
  };

  const connectToROS = () => {
    try {
      // Configuración del servicio ROS con fallback a modo simulado si no está disponible
      const rosInstance = new ROSLIB.Ros({
        url: "ws://localhost:9090"
      });

      rosInstance.on("connection", () => {
        console.log("Conectado al servidor de ROS");
        setIsConnected(true);
      });

      rosInstance.on("error", () => {
        console.log("No se pudo conectar al servidor ROS - Modo simulado activado");
        setIsConnected(false);
        
        // En producción, aquí activaríamos un modo simulado con datos de ejemplo
        // Como solución temporal, usamos datos estáticos
        simulateRobotData();
      });

      rosInstance.on("close", () => {
        console.log("Conexión a ROS cerrada");
        setIsConnected(false);
      });

      setRos(rosInstance);
    } catch (error) {
      console.error("Error iniciando conexión ROS:", error);
      setIsConnected(false);
    }
  };

  // Función para simular datos del robot cuando ROS no está disponible
  const simulateRobotData = () => {
    // Simular batería
    const batteryInterval = setInterval(() => {
      setBatteryPercentage((prev) => {
        // Simular fluctuación de batería
        const change = Math.random() > 0.5 ? 1 : -1;
        const newValue = prev + change;
        if (newValue > 100) return 100;
        if (newValue < 10) return 10;
        return newValue;
      });
      setBatteryStatus("discharging");
      setEstimatedTimeRemaining(calculateEstimatedTime(batteryPercentage, batteryStatus));
    }, 60000); // Actualizar cada minuto

    // Simular posición
    const positionInterval = setInterval(() => {
      setRobotPose((prev) => ({
        x: prev.x + (Math.random() - 0.5) * 0.1,
        y: prev.y + (Math.random() - 0.5) * 0.1
      }));
    }, 5000); // Actualizar cada 5 segundos

    // Limpiar intervalos cuando el componente se desmonte
    return () => {
      clearInterval(batteryInterval);
      clearInterval(positionInterval);
    };
  };

  const reconnect = () => {
    if (ros) {
      ros.close();
    }
    connectToROS();
  };

  useEffect(() => {
    connectToROS();

    return () => {
      if (ros) {
        ros.close();
      }
    };
  }, []);

  useEffect(() => {
    if (!ros || !isConnected) return;

    // Suscripción al tópico de batería
    const batteryTopic = new (ROSLIB as any).Topic({
      ros: ros,
      name: "/battery_status",
      messageType: "sensor_msgs/BatteryState",
    }) as ROSLIBTopic;

    batteryTopic.subscribe((message: any) => {
      const percentage = Math.round(message.percentage);
      setBatteryPercentage(percentage);
      const status = message.power_supply_status === 1 ? "charging" : "discharging";
      setBatteryStatus(status);
      setEstimatedTimeRemaining(calculateEstimatedTime(percentage, status));
    });

    // Suscripción al tópico de alertas del clima
    const weatherTopic = new (ROSLIB as any).Topic({
      ros: ros,
      name: "/weather_alert",
      messageType: "std_msgs/String",
    }) as ROSLIBTopic;

    weatherTopic.subscribe((message: any) => {
      const weatherNotification: Notification = {
        id: Date.now().toString(),
        message: message.data,
        type: "weather",
        timestamp: new Date(),
        read: false,
      };
      setNotifications((prev) => [weatherNotification, ...prev]);
    });

    // Suscripción al tópico de odometría para la posición del robot
    const odomTopic = new (ROSLIB as any).Topic({
      ros: ros,
      name: "/odom", // Asegúrate de que este sea el tópico correcto en tu sistema ROS
      messageType: "nav_msgs/Odometry",
    }) as ROSLIBTopic;

    odomTopic.subscribe((message: any) => {
      setRobotPose({
        x: message.pose.pose.position.x,
        y: message.pose.pose.position.y,
      });
    });

    // Limpieza de las suscripciones al desmontar
    return () => {
      batteryTopic.unsubscribe();
      weatherTopic.unsubscribe();
      odomTopic.unsubscribe();
    };
  }, [ros, isConnected]);

  return (
    <RobotContext.Provider
      value={{
        batteryPercentage,
        batteryStatus,
        estimatedTimeRemaining,
        isConnected,
        notifications,
        robotPose, // Añadimos robotPose al valor del contexto
        markNotificationAsRead,
        reconnect,
      }}
    >
      {children}
    </RobotContext.Provider>
  );
}

export const useRobot = () => useContext(RobotContext);