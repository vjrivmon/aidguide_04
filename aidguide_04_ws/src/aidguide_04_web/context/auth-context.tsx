"use client"

import { createContext, useContext, useState, useEffect, type ReactNode } from "react"
import { useRouter } from "next/navigation"
import authService, { UserResponse, LoginCredentials, RegisterData } from "@/services/auth-service"
import { toast } from "sonner"

type AuthContextType = {
  user: UserResponse | null
  login: (credentials: LoginCredentials) => Promise<boolean>
  register: (data: RegisterData) => Promise<boolean>
  logout: () => void
  isLoading: boolean
  verifyCode: (code: string) => Promise<boolean>
  resendVerificationCode: () => Promise<boolean>
  checkVerification: () => boolean
}

const AuthContext = createContext<AuthContextType | undefined>(undefined)

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<UserResponse | null>(null)
  const [isLoading, setIsLoading] = useState(true)
  const router = useRouter()

  useEffect(() => {
    // Verificar si hay un usuario en localStorage al cargar la página
    const currentUser = authService.getCurrentUser()
    if (currentUser) {
      setUser(currentUser)
    }
    setIsLoading(false)
  }, [])

  const login = async (credentials: LoginCredentials): Promise<boolean> => {
    setIsLoading(true)
    try {
      const response = await authService.login(credentials)
      
      if (!response.usuario || !response.token) {
        toast.error("Credenciales incorrectas")
        setIsLoading(false)
        return false
      }
      
      setUser(response.usuario)
      setIsLoading(false)
      
      toast.success(`Bienvenido/a ${response.usuario.nombre}`)
      
      // Si el usuario es administrador, redirigir al panel de administración
      if (response.usuario.role === "admin") {
        router.push('/admin/dashboard')
      } else if (response.usuario.role === "family") {
        router.push('/family')
      } else {
        router.push('/welcome')
      }
      
      return true
    } catch (error) {
      console.error("Error de inicio de sesión:", error)
      toast.error("Error de inicio de sesión. Verifica tus credenciales.")
      setIsLoading(false)
      return false
    }
  }

  const register = async (data: RegisterData): Promise<boolean> => {
    setIsLoading(true)
    try {
      const response = await authService.register(data)
      setUser(response.usuario)
      setIsLoading(false)
      
      // Después del registro, redirigir a la verificación de correo
      toast.success("Cuenta creada correctamente. Por favor verifica tu correo electrónico.")
      router.push('/verify-email')
      
      return true
    } catch (error) {
      console.error("Error de registro:", error)
      toast.error("Error al crear la cuenta. Inténtalo de nuevo.")
      setIsLoading(false)
      return false
    }
  }

  const verifyCode = async (code: string): Promise<boolean> => {
    if (!user) return false
    
    setIsLoading(true)
    try {
      // Para la simulación, asumimos que el id de usuario es un número 
      // aunque en el tipo sea string
      const userId = parseInt(user.id, 10) || 1
      const success = await authService.verifyEmailCode(userId, code)
      
      if (success) {
        // Actualizar el estado del usuario localmente
        setUser(prev => prev ? { ...prev, verified: true } : null)
        toast.success("Correo electrónico verificado correctamente")
      }
      
      setIsLoading(false)
      return success
    } catch (error) {
      console.error("Error al verificar código:", error)
      toast.error("Error al verificar el código")
      setIsLoading(false)
      return false
    }
  }

  const resendVerificationCode = async (): Promise<boolean> => {
    if (!user) return false
    
    setIsLoading(true)
    try {
      // Para la simulación, asumimos que el id de usuario es un número
      // aunque en el tipo sea string
      const userId = parseInt(user.id, 10) || 1
      const success = await authService.resendVerificationCode(userId)
      
      if (success) {
        toast.success("Código reenviado correctamente")
      }
      
      setIsLoading(false)
      return success
    } catch (error) {
      console.error("Error al reenviar código:", error)
      toast.error("Error al reenviar el código")
      setIsLoading(false)
      return false
    }
  }

  const checkVerification = (): boolean => {
    return authService.isUserVerified()
  }

  const logout = () => {
    authService.logout()
    setUser(null)
    toast.success("Has cerrado sesión correctamente")
    router.push("/")
  }

  return (
    <AuthContext.Provider 
      value={{ 
        user, 
        login, 
        register, 
        logout, 
        isLoading, 
        verifyCode, 
        resendVerificationCode, 
        checkVerification 
      }}
    >
      {children}
    </AuthContext.Provider>
  )
}

export function useAuth() {
  const context = useContext(AuthContext)
  if (context === undefined) {
    throw new Error("useAuth must be used within an AuthProvider")
  }
  return context
}

