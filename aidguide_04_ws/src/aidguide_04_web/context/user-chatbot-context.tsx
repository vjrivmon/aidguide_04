"use client";

import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import { ollamaService, ChatMessage } from '@/services/ollama'; // Asumimos que ollamaService se puede reutilizar o se creará uno específico si es necesario
import { useUserProfileData, UserProfileData } from '@/hooks/useUserProfileData'; // Importar el nuevo hook y tipo

// Interfaz para el contexto del chatbot de usuario
interface UserChatbotContextType {
  messages: ChatMessage[];
  isLoading: boolean;
  isOpen: boolean;
  isAvailable: boolean;
  sendMessage: (content: string) => Promise<void>;
  toggleChatbot: () => void;
  clearMessages: () => void;
}

// Crear el contexto
const UserChatbotContext = createContext<UserChatbotContextType | undefined>(undefined);

// Hook para usar el contexto
export function useUserChatbot() {
  const context = useContext(UserChatbotContext);
  if (!context) {
    throw new Error('useUserChatbot debe ser usado dentro de un UserChatbotProvider');
  }
  return context;
}

// Proveedor del contexto
export function UserChatbotProvider({ children }: { children: ReactNode }) {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [isOpen, setIsOpen] = useState<boolean>(false); // Podrías querer controlar esto de forma diferente para el perfil
  const [isAvailable, setIsAvailable] = useState<boolean>(false);

  // Verificar si Ollama está disponible al cargar el componente
  useEffect(() => {
    const checkAvailability = async () => {
      const available = await ollamaService.isAvailable();
      setIsAvailable(available);
      
      if (messages.length === 0) {
        setMessages([
          {
            role: 'assistant',
            content: '¡Hola! Soy tu asistente de perfil. Puedo darte detalles sobre tus rutas, batería del robot, puntos de gamificación y más. ¿En qué te ayudo hoy?'
          }
        ]);
      }
    };
    
    checkAvailability();
    
    const interval = setInterval(checkAvailability, 5 * 60 * 1000);
    
    return () => clearInterval(interval);
  }, []); // messages.length dependencia eliminada para que no se resetee el mensaje de bienvenida

  // Enviar mensaje al chatbot
  const sendMessage = async (content: string) => {
    if (!content.trim()) return;
    
    const userMessage: ChatMessage = {
      role: 'user',
      content
    };
    
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    
    // Obtener los datos del perfil del usuario ANTES de enviar el mensaje
    const userProfile = useUserProfileData(); // Usar el hook aquí

    try {
      // Convertir los datos del perfil a una cadena JSON para incluirla en el prompt
      // Seleccionar cuidadosamente qué datos son más relevantes o resumirlos si son muy extensos
      const profileContextString = JSON.stringify({
        userName: userProfile.userName,
        robotName: userProfile.robotName,
        batteryLevel: userProfile.batteryLevel,
        robotStatus: userProfile.robotStatus,
        currentLocation: userProfile.currentLocation,
        recentAlerts: userProfile.recentAlerts?.map(a => ({ type: a.type, message: a.message, date: a.timestamp })).slice(0, 3), // Limitar alertas
        gamification: {
          totalPoints: userProfile.gamification.totalPoints,
          level: userProfile.gamification.level,
          pointsToNextLevel: userProfile.gamification.pointsToNextLevel,
        },
        recentActivities: userProfile.recentActivities?.slice(0, 3), // Limitar actividades
        frequentSites: userProfile.frequentSites,
        lastKnownRoute: userProfile.lastKnownRoute,
        // Añadir más campos si se considera esencial y no sobrecarga el prompt
      }, null, 2);


      const contextMessage: ChatMessage = {
        role: 'system',
        content: `Eres un asistente virtual avanzado integrado en el perfil de usuario de la aplicación AidGuide. Tu propósito es responder preguntas sobre el estado y datos del robot conectado, así como información específica del perfil del usuario.

Aquí tienes la información actual del usuario y su robot:
\`\`\`json
${profileContextString}
\`\`\`

Basándote ESTRICTAMENTE en esta información:
1.  Responde preguntas sobre:
    *   Nombre del usuario y del robot.
    *   Porcentaje de batería del robot y su estado actual.
    *   Ubicación actual del robot (si está disponible).
    *   Alertas recientes del robot (tipo, mensaje, fecha).
    *   Puntos de gamificación del usuario, nivel actual y puntos para el siguiente nivel.
    *   Actividades recientes del usuario (descripción, fecha).
    *   Sitios que el usuario suele visitar.
    *   Información sobre la última ruta conocida (nombre, fecha, origen, destino, distancia).
    *   Disponibilidad de recompensas o desafíos (basado en los puntos y actividades).
2.  Si te preguntan por rutas de un día específico y no tienes esa información exacta en 'lastKnownRoute' o 'recentActivities', indica que solo tienes información general o la más reciente.
3.  Si te preguntan por algo fuera de la información proporcionada (ej. el tiempo atmosférico detallado, noticias, etc.), indica amablemente que no tienes acceso a esa información y que te centres en los datos del perfil y del robot.
4.  Sé conciso, amigable y directo.
5.  Responde SIEMPRE en español.
6.  No inventes información. Si no tienes un dato, dilo claramente.
7.  Para preguntas sobre "puntos para canjear", refiérete a 'totalPoints' en 'gamification'.
8.  Para "sitios que suele visitar más", usa 'frequentSites'.
9.  Para "ruta de un día x", revisa 'lastKnownRoute' y 'recentActivities' y responde con la información más relevante que encuentres, especificando la fecha si es diferente a la solicitada.`
      };
      
      if (isAvailable) {
        const allMessages = [contextMessage, ...messages, userMessage]; // Se envían todos los mensajes para mantener contexto
        // Usaremos una nueva ruta '/api/user-chat' para este chatbot
        const response = await fetch('/api/user-chat', { // IMPORTANTE: Nueva ruta API
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ messages: allMessages }), // Enviar todos los mensajes
        });

        if (!response.ok) {
          // Manejo de errores específico si es necesario
          const errorText = await response.text();
          console.error("Error desde /api/user-chat:", errorText);
          throw new Error(`Error de API: ${response.status}`);
        }
        
        const data = await response.json();
        
        setMessages(prev => [
          ...prev,
          { role: 'assistant', content: data.message.content }
        ]);

      } else {
        setMessages(prev => [
          ...prev,
          {
            role: 'assistant',
            content: 'Lo siento, el servicio de chat para el perfil de usuario no está disponible. Verifica la conexión del servidor Ollama.'
          }
        ]);
      }
    } catch (error) {
      console.error('Error al enviar mensaje al chatbot de usuario:', error);
      setMessages(prev => [
        ...prev,
        {
          role: 'assistant',
          content: 'Hubo un problema al procesar tu solicitud. Por favor, inténtalo más tarde.'
        }
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const toggleChatbot = () => {
    setIsOpen(prev => !prev);
  };

  const clearMessages = () => {
    setMessages([
      {
        role: 'assistant',
        content: '¡Hola! Soy tu asistente de perfil. Puedo darte detalles sobre tus rutas, batería del robot, puntos de gamificación y más. ¿En qué te ayudo hoy?'
      }
    ]);
  };

  return (
    <UserChatbotContext.Provider
      value={{
        messages,
        isLoading,
        isOpen,
        isAvailable,
        sendMessage,
        toggleChatbot,
        clearMessages
      }}
    >
      {children}
    </UserChatbotContext.Provider>
  );
} 