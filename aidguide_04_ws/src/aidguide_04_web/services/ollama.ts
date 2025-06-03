/**
 * Servicio para interactuar con la API del chatbot
 */

export interface ChatMessage {
  role: 'user' | 'assistant' | 'system';
  content: string;
}

export interface ChatResponse {
  message: ChatMessage;
  done: boolean;
}

/**
 * Clase que proporciona métodos para interactuar con la API del chatbot
 */
export class OllamaService {
  /**
   * Envía un mensaje al chatbot y devuelve la respuesta
   * @param messages - Array de mensajes anteriores para mantener el contexto
   * @returns Promise con la respuesta del chatbot
   */
  async chat(messages: ChatMessage[]): Promise<string> {
    try {
      const response = await fetch('/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          messages,
        }),
      });

      if (!response.ok) {
        if (response.status === 503) {
          return 'Lo siento, el servicio de chat no está disponible en este momento. Por favor, asegúrate de que el servidor Ollama esté en funcionamiento.';
        }
        throw new Error(`Error de API: ${response.status}`);
      }

      const data = await response.json();
      return data.message.content;
    } catch (error) {
      console.error('Error al comunicarse con el chatbot:', error);
      return 'Lo siento, no puedo responder en este momento. Por favor, intenta de nuevo más tarde o verifica la conexión con el servidor.';
    }
  }

  /**
   * Verifica si el servicio del chatbot está disponible
   * @returns Promise<boolean> - true si el servicio está disponible
   */
  async isAvailable(): Promise<boolean> {
    try {
      const response = await fetch('/api/chat', {
        method: 'GET',
      });
      
      if (response.ok) {
        const data = await response.json();
        return data.status === 'available';
      }
      
      return false;
    } catch (error) {
      console.error('Error al verificar disponibilidad del chatbot:', error);
      return false;
    }
  }
}

// Instancia singleton del servicio
export const ollamaService = new OllamaService(); 