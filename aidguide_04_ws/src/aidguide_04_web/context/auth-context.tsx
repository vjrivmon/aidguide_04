"use client"

import { createContext, useContext, useState, useEffect, type ReactNode } from "react"
import { useRouter } from "next/navigation"

type User = {
  id: string
  name: string
  email: string
  role: "user" | "admin" | "family"
}

type AuthContextType = {
  user: User | null
  login: (email: string, password: string) => Promise<boolean>
  logout: () => void
  isLoading: boolean
}

const AuthContext = createContext<AuthContextType | undefined>(undefined)

export function AuthProvider({ children }: { children: ReactNode }) {
  const [user, setUser] = useState<User | null>(null)
  const [isLoading, setIsLoading] = useState(true)
  const router = useRouter()

  useEffect(() => {
    // Verificar si hay un usuario en localStorage al cargar la página
    const storedUser = localStorage.getItem("aidguide-user")
    if (storedUser) {
      setUser(JSON.parse(storedUser))
    }
    setIsLoading(false)
  }, [])

  const login = async (email: string, password: string): Promise<boolean> => {
    setIsLoading(true)

    // Simulación de autenticación
    return new Promise((resolve) => {
      setTimeout(() => {
        if (email === "admin@aidguide.com" && password === "admin123") {
          const adminUser = {
            id: "admin-1",
            name: "Administrador",
            email: "admin@aidguide.com",
            role: "admin" as const,
          }
          setUser(adminUser)
          localStorage.setItem("aidguide-user", JSON.stringify(adminUser))
          resolve(true)
        } else if (email === "user@aidguide.com" && password === "user123") {
          const regularUser = {
            id: "user-1",
            name: "María García",
            email: "user@aidguide.com",
            role: "user" as const,
          }
          setUser(regularUser)
          localStorage.setItem("aidguide-user", JSON.stringify(regularUser))
          resolve(true)
        } else if (email === "family@aidguide.com" && password === "family123") {
          const regularUser = {
            id: "user-2",
            name: "Juana García",
            email: "family@aidguide.com",
            role: "family" as const,
          }
          setUser(regularUser)
          localStorage.setItem("aidguide-user", JSON.stringify(regularUser))
          resolve(true)
        }
        
        
        else {
          resolve(false)
        }
        setIsLoading(false)
      }, 1000)
    })
  }

  const logout = () => {
    setUser(null)
    localStorage.removeItem("aidguide-user")
    router.push("/")
  }

  return <AuthContext.Provider value={{ user, login, logout, isLoading }}>{children}</AuthContext.Provider>
}

export function useAuth() {
  const context = useContext(AuthContext)
  if (context === undefined) {
    throw new Error("useAuth must be used within an AuthProvider")
  }
  return context
}

