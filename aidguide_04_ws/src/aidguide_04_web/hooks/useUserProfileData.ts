"use client";

import { useAuth } from "@/context/auth-context";
import { useRobot } from "@/context/robot-context";
import { RobotNotification } from "@/types/robot"; // Asumiendo que este tipo existe

// --- Tipos de Datos (simulados o a ser reemplazados por datos reales) ---

interface UserPreferences {
  voiceVolume: number;
  speechRate: number;
  notificationsEnabled: boolean;
  highContrastMode: boolean;
  largeText: boolean;
}

interface UserGamification {
  totalPoints: number;
  level: number;
  completedChallenges: number;
  availableDiscounts: number;
  levelProgress: number;
  pointsToNextLevel: number;
}

interface RecentActivity {
  type: string;
  description: string;
  date: string; // Podría ser Date para mejor manejo
  // icon?: JSX.Element; // Los iconos son para UI, no para el LLM directamente
}

interface UpcomingAppointment {
  title: string;
  date: string;
  time: string;
  location: string;
}

interface Challenge {
  id: number;
  title: string;
  description: string;
  points: number;
  progress: number; // porcentaje
  category: string;
}

interface Reward {
  id: number;
  title: string;
  description: string;
  points: number; // Puntos necesarios para canjear
  provider: string;
  expiry?: string;
}

interface Achievement {
  id: number;
  title: string;
  description: string;
  date: string;
}

export interface UserProfileData {
  // Del useAuth
  userName: string | null | undefined;
  userEmail: string | null | undefined;
  // Del useRobot
  robotName: string | null;
  robotStatus: string | null;
  batteryLevel: number | null;
  currentLocation: { latitude: number; longitude: number } | null; // Asumiendo estructura
  recentAlerts: RobotNotification[]; // Usando las notificaciones como alertas
  // Datos que estaban hardcodeados en las páginas
  phone: string;
  address: string;
  emergencyContact: string;
  preferences: UserPreferences;
  gamification: UserGamification;
  recentActivities: RecentActivity[];
  upcomingAppointments: UpcomingAppointment[];
  challenges: Challenge[];
  availableRewards: Reward[];
  unlockedAchievements: Achievement[];
  // Podríamos añadir más campos según sea necesario
  // Por ejemplo, sitios más visitados, rutas específicas, etc.
  // Esto requeriría una lógica más compleja o un backend para rastrear.
  // Para este ejemplo, nos enfocaremos en la información ya disponible.
  frequentSites?: string[]; // Ejemplo: ['Casa', 'Trabajo', 'Supermercado']
  lastKnownRoute?: {
    name: string;
    date: string;
    origin: string;
    destination: string;
    distance?: string; // ej: '2.5 km'
  };
}

export function useUserProfileData(): UserProfileData {
  const { user } = useAuth();
  const { robotDetails, notifications, battery, location } = useRobot(); // Asumiendo que location está disponible en useRobot

  // Datos simulados (reemplazar con lógica real o llamadas a API si es necesario)
  // Estos datos se tomarían de los archivos de perfil que analizamos.

  const simulatedUserData = {
    name: "María García", // Este podría venir de user.displayName o similar
    email: "maria.garcia@example.com", // Este podría venir de user.email
    phone: "+34 612 345 678",
    address: "Calle Principal 123, Valencia",
    emergencyContact: "Juan García - +34 698 765 432",
    preferences: {
      voiceVolume: 80,
      speechRate: 60,
      notificationsEnabled: true,
      highContrastMode: false,
      largeText: true,
    },
    gamification: {
      totalPoints: 1250,
      level: 3,
      completedChallenges: 8,
      availableDiscounts: 2,
      levelProgress: 65,
      pointsToNextLevel: 350,
    },
    recentActivities: [
      {
        type: "route",
        description: "Ruta completada: Casa - Trabajo",
        date: "Hoy, 09:15",
      },
      {
        type: "settings",
        description: "Configuración actualizada: Volumen de voz",
        date: "Ayer, 18:30",
      },
      {
        type: "notification",
        description: "Alerta: Batería baja (20%)",
        date: "Ayer, 16:45", // Esto podría venir de 'notifications' del useRobot
      },
      {
        type: "route",
        description: "Ruta completada: Trabajo - Supermercado - Casa",
        date: "Hace 2 días, 19:20",
      },
    ],
    upcomingAppointments: [
      {
        title: "Mantenimiento preventivo",
        date: "En 3 días",
        time: "10:00 - 11:00",
        location: "Centro de servicio AidGuide",
      },
    ],
    challenges: [
      {
        id: 1,
        title: "Caminante diario",
        description: "Completa 15.000 pasos en un día",
        points: 150,
        progress: 65,
        category: "daily",
      },
      {
        id: 2,
        title: "Explorador urbano",
        description: "Completa 3 rutas diferentes en una semana",
        points: 200,
        progress: 33,
        category: "weekly",
      },
    ],
    availableRewards: [
      {
        id: 1,
        title: "10% de descuento en transporte público",
        description: "Vale válido para un viaje en cualquier transporte público",
        points: 500,
        provider: "Metro Valencia",
      },
      {
        id: 2,
        title: "5€ en tu cafetería favorita",
        description: "Descuento aplicable en cualquier compra superior a 10€",
        points: 300,
        provider: "Café Central",
      },
    ],
    unlockedAchievements: [
      {
        id: 1,
        title: "Primeros pasos",
        description: "Completaste tu primera ruta con AidGuide",
        date: "Hace 1 semana",
      },
    ],
    // Datos adicionales que el chatbot podría usar
    frequentSites: ["Casa", "Trabajo", "Supermercado El Árbol", "Parque Central"],
    lastKnownRoute: {
        name: "Paseo por el río",
        date: "Ayer por la tarde",
        origin: "Puente de las Flores",
        destination: "Ciudad de las Artes y las Ciencias",
        distance: "3 km"
    }
  };

  // Filtrar notificaciones relevantes para "recentAlerts"
  // Por ejemplo, solo las no leídas o las de tipo 'error' o 'warning'
  const recentAlerts = notifications.filter(
    (n) => !n.read && (n.type === 'battery' || n.type === 'system' || n.type === 'obstacle_detection' || n.type === 'emergency')
  );


  return {
    userName: user?.displayName || user?.email || simulatedUserData.name,
    userEmail: user?.email || simulatedUserData.email,
    robotName: robotDetails?.name || "Robot Guía",
    robotStatus: robotDetails?.status || "Desconocido", // Asumimos que robotDetails tiene 'status'
    batteryLevel: battery !== null ? battery : null, // Viene de useRobot
    currentLocation: location || null, // Viene de useRobot, asegurar que la estructura coincide
    recentAlerts: recentAlerts, // Notificaciones filtradas de useRobot
    phone: simulatedUserData.phone,
    address: simulatedUserData.address,
    emergencyContact: simulatedUserData.emergencyContact,
    preferences: simulatedUserData.preferences,
    gamification: simulatedUserData.gamification,
    recentActivities: simulatedUserData.recentActivities,
    upcomingAppointments: simulatedUserData.upcomingAppointments,
    challenges: simulatedUserData.challenges,
    availableRewards: simulatedUserData.availableRewards,
    unlockedAchievements: simulatedUserData.unlockedAchievements,
    frequentSites: simulatedUserData.frequentSites,
    lastKnownRoute: simulatedUserData.lastKnownRoute,
  };
} 