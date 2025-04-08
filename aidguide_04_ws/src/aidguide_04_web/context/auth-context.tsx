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
      setUser(response.usuario)
      setIsLoading(false)
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
      return true
    } catch (error) {
      console.error("Error de registro:", error)
      toast.error("Error al crear la cuenta. Inténtalo de nuevo.")
      setIsLoading(false)
      return false
    }
  }

  const logout = () => {
    authService.logout()
    setUser(null)
    router.push("/")
  }

  return (
    <AuthContext.Provider value={{ user, login, register, logout, isLoading }}>
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

