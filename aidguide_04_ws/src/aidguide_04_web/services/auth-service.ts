// Tipos para el servicio de autenticación
export interface LoginCredentials {
  email: string
  password: string
}

export interface RegisterData {
  nombre: string
  email: string
  password: string
}

export interface UserResponse {
  id: number
  nombre: string
  email: string
  token?: string
  verified?: boolean
}

export interface AuthResponse {
  usuario: UserResponse
  token: string
}

// URL base de la API
const API_URL = 'http://localhost:3000/api';

// Servicio de autenticación
const authService = {
  // Iniciar sesión
  async login(credentials: LoginCredentials): Promise<AuthResponse> {
    try {
      const response = await fetch(`${API_URL}/auth/login`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(credentials),
      });

      if (!response.ok) {
        throw new Error('Error de inicio de sesión');
      }

      const data = await response.json();
      // Guardar el token en localStorage
      localStorage.setItem('authToken', data.token);
      localStorage.setItem('user', JSON.stringify(data.usuario));
      
      return data;
    } catch (error) {
      console.error('Error en login:', error);
      throw error;
    }
  },

  // Registrar un nuevo usuario
  async register(data: RegisterData): Promise<AuthResponse> {
    try {
      const response = await fetch(`${API_URL}/auth/register`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        throw new Error('Error de registro');
      }

      const responseData = await response.json();
      // Guardar el token en localStorage
      localStorage.setItem('authToken', responseData.token);
      localStorage.setItem('user', JSON.stringify(responseData.usuario));
      
      return responseData;
    } catch (error) {
      console.error('Error en register:', error);
      throw error;
    }
  },

  // Verificar el código de 6 dígitos
  async verifyEmailCode(userId: number, code: string): Promise<boolean> {
    try {
      const token = this.getToken();
      
      if (!token) {
        throw new Error('No hay token de autenticación');
      }
      
      const response = await fetch(`${API_URL}/auth/verify-email`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({ userId, code }),
      });

      if (!response.ok) {
        throw new Error('Código de verificación inválido');
      }

      const data = await response.json();
      
      // Actualizar el estado del usuario en localStorage
      const user = this.getCurrentUser();
      if (user) {
        user.verified = true;
        localStorage.setItem('user', JSON.stringify(user));
      }
      
      return data.success;
    } catch (error) {
      console.error('Error al verificar código:', error);
      throw error;
    }
  },

  // Solicitar reenvío del código de verificación
  async resendVerificationCode(userId: number): Promise<boolean> {
    try {
      const token = this.getToken();
      
      if (!token) {
        throw new Error('No hay token de autenticación');
      }
      
      const response = await fetch(`${API_URL}/auth/resend-verification`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({ userId }),
      });

      if (!response.ok) {
        throw new Error('Error al reenviar código');
      }

      const data = await response.json();
      return data.success;
    } catch (error) {
      console.error('Error al reenviar código:', error);
      throw error;
    }
  },

  // Obtener el usuario actual desde localStorage
  getCurrentUser(): UserResponse | null {
    if (typeof window === 'undefined') {
      return null; // Estamos en el servidor
    }
    
    const userStr = localStorage.getItem('user');
    if (!userStr) {
      return null;
    }
    
    try {
      return JSON.parse(userStr) as UserResponse;
    } catch (error) {
      console.error('Error al parsear el usuario:', error);
      return null;
    }
  },

  // Verificar si el usuario está verificado
  isUserVerified(): boolean {
    const user = this.getCurrentUser();
    return user?.verified === true;
  },

  // Cerrar sesión
  logout(): void {
    localStorage.removeItem('authToken');
    localStorage.removeItem('user');
  },

  // Obtener el token de autenticación
  getToken(): string | null {
    if (typeof window === 'undefined') {
      return null; // Estamos en el servidor
    }
    return localStorage.getItem('authToken');
  }
};

export default authService; 