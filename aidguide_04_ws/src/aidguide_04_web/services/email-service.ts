// Servicio para envío de correos y verificación de códigos
import authService from './auth-service';

// URL base de la API
const API_URL = 'http://localhost:3000/api';

// Tipos para el servicio de email
export interface EmailVerificationRequest {
  email: string;
  userId: number;
}

export interface VerificationResponse {
  success: boolean;
  message: string;
}

const emailService = {
  // Enviar correo con código de verificación
  async sendVerificationCode(data: EmailVerificationRequest): Promise<VerificationResponse> {
    try {
      const token = authService.getToken();
      
      if (!token) {
        throw new Error('No hay token de autenticación');
      }
      
      const response = await fetch(`${API_URL}/email/send-verification`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Error al enviar código de verificación');
      }

      return await response.json();
    } catch (error) {
      console.error('Error al enviar código de verificación:', error);
      throw error;
    }
  },

  // Verificar código
  async verifyCode(userId: number, code: string): Promise<VerificationResponse> {
    try {
      const token = authService.getToken();
      
      if (!token) {
        throw new Error('No hay token de autenticación');
      }
      
      const response = await fetch(`${API_URL}/email/verify-code`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({ userId, code }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Código de verificación inválido');
      }

      return await response.json();
    } catch (error) {
      console.error('Error al verificar código:', error);
      throw error;
    }
  },
  
  // Reenviar código de verificación
  async resendVerificationCode(userId: number): Promise<VerificationResponse> {
    try {
      const token = authService.getToken();
      
      if (!token) {
        throw new Error('No hay token de autenticación');
      }
      
      const response = await fetch(`${API_URL}/email/resend-code`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({ userId }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Error al reenviar código');
      }

      return await response.json();
    } catch (error) {
      console.error('Error al reenviar código:', error);
      throw error;
    }
  }
};

export default emailService; 