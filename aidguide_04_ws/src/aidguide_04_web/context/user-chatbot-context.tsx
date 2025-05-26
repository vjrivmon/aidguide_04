"use client";

import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import { ollamaService, ChatMessage } from '@/services/ollama'; // Asumimos que ollamaService se puede reutilizar o se creará uno específico si es necesario

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
      // Asumimos que el servicio ollama tiene un método isAvailable o usamos /api/user-chat para verificar
      const available = await ollamaService.isAvailable(); // Podría necesitar una URL diferente o un nuevo servicio
      setIsAvailable(available);
      
      // Agregar mensaje de bienvenida específico del perfil de usuario
      if (messages.length === 0) {
        setMessages([
          {
            role: 'assistant',
            content: '¡Hola! Soy tu asistente del perfil de robot. Puedo informarte sobre la batería, alertas y más. ¿Qué necesitas saber?'
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
    
    try {
      const contextMessage: ChatMessage = {
        role: 'system',
        content: '''Eres un asistente virtual integrado en el perfil de usuario de la aplicación AidGuide. Tu propósito es responder preguntas sobre el estado y datos del robot conectado. Esto incluye, pero no se limita a: porcentaje de batería del robot, alertas activas, notificaciones recientes, descripción de imágenes detectadas por el robot, sugerencias para la mejor ruta basada en datos del robot, y otra información relevante del perfil del usuario o del estado del robot. Debes basar tus respuestas en la información que te sea proporcionada o que puedas inferir del contexto del perfil del usuario. Responde SIEMPRE en español. Sé conciso y directo.''''
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
        content: '¡Hola! Soy tu asistente del perfil de robot. ¿En qué puedo ayudarte?'
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