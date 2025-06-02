"use client";

import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import { ollamaService, ChatMessage } from '@/services/ollama';
import ROSService from "@/services/ros-service";

// Interfaz para el contexto del chatbot
interface ChatbotContextType {
  messages: ChatMessage[];
  isLoading: boolean;
  isOpen: boolean;
  isAvailable: boolean;
  sendMessage: (content: string) => Promise<void>;
  toggleChatbot: () => void;
  clearMessages: () => void;
}

// Contexto con valores por defecto
const ChatbotContext = createContext<ChatbotContextType>({
  messages: [],
  isLoading: false,
  isOpen: false,
  isAvailable: false,
  sendMessage: async () => {},
  toggleChatbot: () => {},
  clearMessages: () => {},
});

// Hook personalizado para usar el contexto
export const useChatbot = () => useContext(ChatbotContext);

// Proveedor del contexto
export function ChatbotProvider({ children }: { children: ReactNode }) {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [isOpen, setIsOpen] = useState<boolean>(false);
  const [isAvailable, setIsAvailable] = useState<boolean>(false);
  const [rosService, setRosService] = useState<ROSService | null>(null);
  const [isRosConnected, setIsRosConnected] = useState<boolean>(false);

  // Inicializar el servicio ROS y verificar conexión
  useEffect(() => {
    const service = ROSService.getInstance();
    setRosService(service);
    
    const handleConnectionStatus = (connected: boolean) => {
      setIsRosConnected(connected);
    };
    
    service.addConnectionListener(handleConnectionStatus);
    
    return () => {
      service.removeConnectionListener(handleConnectionStatus);
    };
  }, []);

  // Verificar si Ollama está disponible al cargar el componente
  useEffect(() => {
    const checkAvailability = async () => {
      const available = await ollamaService.isAvailable();
      setIsAvailable(available);
      
      // Agregar mensaje de bienvenida
      if (messages.length === 0) {
        setMessages([
          {
            role: 'assistant',
            content: '¡Hola! Soy el asistente virtual de AidGuide. ¿En qué puedo ayudarte hoy?'
          }
        ]);
      }
    };
    
    checkAvailability();
    
    // Verificar disponibilidad cada 5 minutos
    const interval = setInterval(checkAvailability, 5 * 60 * 1000);
    
    return () => clearInterval(interval);
  }, []);

  // Manejar comando especial de navegación
  const handleNavigationCommand = (content: string) => {
    const navigationCommand = "ve en simulación al supermercado";
    
    if (content.toLowerCase().includes(navigationCommand.toLowerCase())) {
      if (rosService && isRosConnected) {
        const success = rosService.startWaypointFollowing();
        if (success) {
          return {
            handled: true,
            response: "Iniciando navegación en simulación hacia el supermercado. El robot comenzará a moverse ahora."
          };
        } else {
          return {
            handled: true,
            response: "No se pudo iniciar la navegación. Hay un problema con el servicio ROS."
          };
        }
      } else {
        return {
          handled: true,
          response: "No puedo iniciar la navegación porque no hay conexión con ROS. Verifica que el servidor ROS esté en funcionamiento."
        };
      }
    }
    
    return { handled: false };
  };

  // Enviar mensaje al chatbot
  const sendMessage = async (content: string) => {
    if (!content.trim()) return;
    
    // Agregar mensaje del usuario
    const userMessage: ChatMessage = {
      role: 'user',
      content
    };
    
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    
    try {
      // Primero verificamos si es un comando de navegación
      const navigationResult = handleNavigationCommand(content);
      
      if (navigationResult.handled) {
        // Si es un comando de navegación, agregamos la respuesta predefinida
        setMessages(prev => [
          ...prev,
          {
            role: 'assistant',
            content: navigationResult.response
          }
        ]);
        setIsLoading(false);
        return;
      }
      
      // Si no es un comando de navegación, continuamos con el flujo normal
      const contextMessage: ChatMessage = {
        role: 'system',
        content: 'Eres un asistente virtual para la aplicación AidGuide, un robot guía para personas invidentes. Tu objetivo es proporcionar información útil sobre la aplicación, el producto, y ayudar a navegar por la interfaz. IMPORTANTE: Debes responder SIEMPRE en español. Tus respuestas deben ser concisas, claras y útiles. Limita tus respuestas a información sobre AidGuide y su uso.'
      };
      
      if (isAvailable) {
        // Enviar mensaje y obtener respuesta
        const allMessages = [contextMessage, ...messages, userMessage];
        const response = await ollamaService.chat(allMessages);
        
        // Agregar respuesta del asistente
        setMessages(prev => [
          ...prev,
          {
            role: 'assistant',
            content: response
          }
        ]);
      } else {
        // Mensaje de error en español cuando Ollama no está disponible
        setMessages(prev => [
          ...prev,
          {
            role: 'assistant',
            content: 'Lo siento, el servicio de chat no está disponible en este momento. Por favor, verifica que el servidor Ollama esté ejecutándose correctamente e intenta de nuevo más tarde.'
          }
        ]);
      }
    } catch (error) {
      console.error('Error al enviar mensaje:', error);
      
      // Mensaje de error
      setMessages(prev => [
        ...prev,
        {
          role: 'assistant',
          content: 'Lo siento, ha ocurrido un error al procesar tu mensaje. Por favor, inténtalo de nuevo más tarde.'
        }
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  // Alternar visibilidad del chatbot
  const toggleChatbot = () => {
    setIsOpen(prev => !prev);
  };

  // Limpiar historial de mensajes
  const clearMessages = () => {
    setMessages([
      {
        role: 'assistant',
        content: '¡Hola! Soy el asistente virtual de AidGuide. ¿En qué puedo ayudarte hoy?'
      }
    ]);
  };

  // Proporcionar el contexto
  return (
    <ChatbotContext.Provider
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
    </ChatbotContext.Provider>
  );
} 