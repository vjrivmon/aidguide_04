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
  id: string
  nombre: string
  email: string
  token?: string
  verified?: boolean
  role?: string
}

export interface AuthResponse {
  usuario: UserResponse
  token: string
}

// URL base de la API
const API_URL = 'http://localhost:3000/api';

// Tipos de usuario según la base de datos
interface DBUser {
  id_usuario: number
  nombre: string
  apellidos: string
  correo: string
  id_rol: number
}

interface DBAuthData {
  id_autenticacion: number
  email: string
  password_hash: string
  token_acceso: string
  verificado: boolean
}

// Servicio de autenticación
const authService = {
  // Iniciar sesión con simulación
  async login(credentials: LoginCredentials): Promise<AuthResponse> {
    // Simulación de autenticación
    return new Promise((resolve) => {
      setTimeout(() => {
        if (credentials.email === "admin@aidguide.com" && credentials.password === "admin123") {
          const adminUser = {
            id: "admin-1",
            nombre: "Administrador",
            email: "admin@aidguide.com",
            role: "admin" as const,
          }
          this.setUser(adminUser)
          localStorage.setItem("aidguide-user", JSON.stringify(adminUser))
          resolve({
            usuario: adminUser,
            token: "admin-token-123"
          })
        } else if (credentials.email === "user@aidguide.com" && credentials.password === "user123") {
          const regularUser = {
            id: "user-1",
            nombre: "María García",
            email: "user@aidguide.com",
            role: "user" as const,
          }
          this.setUser(regularUser)
          localStorage.setItem("aidguide-user", JSON.stringify(regularUser))
          resolve({
            usuario: regularUser,
            token: "user-token-123"
          })
        } else if (credentials.email === "family@aidguide.com" && credentials.password === "family123") {
          const regularUser = {
            id: "user-2",
            nombre: "Juana García",
            email: "family@aidguide.com",
            role: "family" as const,
          }
          this.setUser(regularUser)
          localStorage.setItem("aidguide-user", JSON.stringify(regularUser))
          resolve({
            usuario: regularUser,
            token: "family-token-123"
          })
        } else {
          resolve({
            usuario: null,
            token: null
          } as any)
        }
        this.setIsLoading(false)
      }, 1000)
    });
  },

  // Mock función para simular el estado de carga
  setIsLoading(value: boolean) {
    // Esta función sería implementada en un contexto real
    console.log(`Loading state: ${value}`);
  },

  // Mock función para establecer el usuario actual
  setUser(user: UserResponse) {
    // Esta función sería implementada en un contexto real
    console.log(`Setting user: ${JSON.stringify(user)}`);
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
    
    // Primero intentar con la clave 'aidguide-user' (para el código viejo)
    let userStr = localStorage.getItem('aidguide-user');
    if (!userStr) {
      // Si no existe, intentar con la clave 'user' (para el código nuevo)
      userStr = localStorage.getItem('user');
      if (!userStr) {
        return null;
      }
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
    localStorage.removeItem('aidguide-user');
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