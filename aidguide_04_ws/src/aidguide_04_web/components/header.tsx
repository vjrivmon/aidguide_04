"use client"

import { useState } from "react"
import Link from "next/link"
import Image from "next/image"
import { Menu, X, User, LogOut, Trophy } from "lucide-react"
import { useAuth } from "@/context/auth-context"

export default function Header() {
  const [isMenuOpen, setIsMenuOpen] = useState(false)
  const { user, logout } = useAuth()

  return (
    <header className="bg-white">
      <div className="container-custom py-4">
        <div className="flex items-center justify-between">
          <Link href="/" className="flex items-center">
            <Image src="/logo.svg" alt="AidGuide Logo" width={100} height={76} className="h-12 w-auto" />
            <span className="ml-2 text-xl font-bold text-button">AidGuide</span>
          </Link>

          {/* Desktop Navigation */}
          <nav className="hidden md:flex items-center space-x-6">
            {/*<Link href="/" className="text-text hover:text-button transition-colors">
              Inicio
            </Link>*/}

            {/* Enlaces para todos los usuarios */}
            {!user && (
              <>
                <Link href="/about" className="text-text hover:text-button transition-colors">
                  Quiénes Somos
                </Link>
                <Link href="/product" className="text-text hover:text-button transition-colors">
                  Producto
                </Link>
                <Link href="/faq" className="text-text hover:text-button transition-colors">
                  FAQ
                </Link>
              </>
            )}

            {/* Enlaces para usuarios autenticados */}
            {user && user.role === "user" && (
              <>
                <Link href="/welcome" className="text-text hover:text-button transition-colors">
                  Inicio
                </Link>
                <Link href="/routes" className="text-text hover:text-button transition-colors">
                  Rutas
                </Link>
                <Link href="/robot-feed" className="text-text hover:text-button transition-colors">
                  Imágenes
                </Link>
                <Link href="/profile/gamification" className="text-text hover:text-button transition-colors flex items-center">
                  <Trophy size={18} className="mr-1" />
                  Gamificación
                </Link>
                <Link href="/profile" className="text-text hover:text-button transition-colors">
                  Perfil
                </Link>
              </>
            )}

            {/* Enlaces para familiares */}
            {user && user.role === "family" && (
              <>
                <Link href="/family" className="text-text hover:text-button transition-colors">
                  Inicio
                </Link>
                <Link href="/family/profile" className="text-text hover:text-button transition-colors">
                  Perfil
                </Link>
              </>
            )}

            {/* Enlaces para administradores */}
            {user && user.role === "admin" && (
              <Link href="/admin/dashboard" className="text-text hover:text-button transition-colors">
                Panel Admin
              </Link>
            )}

            {/* Botones de autenticación */}
            {!user ? (
              <>
                <Link href="/login" className="btn-secondary">
                  Iniciar Sesión
                </Link>
                <Link href="/register" className="btn-primary">
                  Registrarse
                </Link>
              </>
            ) : (
              <div className="flex items-center space-x-4">
                <div className="flex items-center">
                  <div className="w-8 h-8 bg-button rounded-full flex items-center justify-center text-white">
                    <User size={18} />
                  </div>
                  <span className="ml-2 font-medium">{user.name}</span>
                </div>
                <button onClick={logout} className="btn-secondary flex items-center">
                  <LogOut size={18} className="mr-2" />
                  Cerrar Sesión
                </button>
              </div>
            )}
          </nav>

          {/* Mobile Menu Button */}
          <button
            className="md:hidden text-text"
            onClick={() => setIsMenuOpen(!isMenuOpen)}
            aria-label={isMenuOpen ? "Cerrar menú" : "Abrir menú"}
          >
            {isMenuOpen ? <X size={24} /> : <Menu size={24} />}
          </button>
        </div>

        {/* Mobile Navigation */}
        {isMenuOpen && (
          <nav className="md:hidden py-4 flex flex-col space-y-4">
            {/*<Link
              href="/"
              className="text-text hover:text-button transition-colors"
              onClick={() => setIsMenuOpen(false)}
            >
              Inicio
            </Link>*/}

            {/* Enlaces para todos los usuarios */}
            {!user && (
              <>
                <Link
                  href="/about"
                  className="text-text hover:text-button transition-colors"
                  onClick={() => setIsMenuOpen(false)}
                >
                  Quiénes Somos
                </Link>
                <Link
                  href="/product"
                  className="text-text hover:text-button transition-colors"
                  onClick={() => setIsMenuOpen(false)}
                >
                  Producto
                </Link>
                <Link
                  href="/faq"
                  className="text-text hover:text-button transition-colors"
                  onClick={() => setIsMenuOpen(false)}
                >
                  FAQ
                </Link>
              </>
            )}

            {/* Enlaces para usuarios autenticados */}
            {user && user.role === "user" && (
              <>
                <Link
                  href="/welcome"
                  className="text-text hover:text-button transition-colors"
                  onClick={() => setIsMenuOpen(false)}
                >
                  Inicio
                </Link>
                <Link
                  href="/routes"
                  className="text-text hover:text-button transition-colors"
                  onClick={() => setIsMenuOpen(false)}
                >
                  Rutas
                </Link>
                <Link
                  href="/robot-feed"
                  className="text-text hover:text-button transition-colors"
                  onClick={() => setIsMenuOpen(false)}
                >
                  Imágenes
                </Link>
                <Link
                  href="/profile/gamification"
                  className="text-text hover:text-button transition-colors flex items-center"
                  onClick={() => setIsMenuOpen(false)}
                >
                  <Trophy size={18} className="mr-1" />
                  Gamificación
                </Link>
                <Link
                  href="/profile"
                  className="text-text hover:text-button transition-colors"
                  onClick={() => setIsMenuOpen(false)}
                >
                  Perfil
                </Link>
              </>
            )}

            {/* Enlaces para familiares */}
            {user && user.role === "family" && (
              <>
                <Link
                  href="/family"
                  className="text-text hover:text-button transition-colors"
                  onClick={() => setIsMenuOpen(false)}
                >
                  Inicio
                </Link>
                <Link
                  href="/family/profile"
                  className="text-text hover:text-button transition-colors"
                  onClick={() => setIsMenuOpen(false)}
                >
                  Perfil
                </Link>
              </>
            )}

            {/* Enlaces para administradores */}
            {user && user.role === "admin" && (
              <Link
                href="/admin/dashboard"
                className="text-text hover:text-button transition-colors"
                onClick={() => setIsMenuOpen(false)}
              >
                Panel Admin
              </Link>
            )}

            {/* Botones de autenticación */}
            {!user ? (
              <div className="flex flex-col space-y-2 pt-2">
                <Link href="/login" className="btn-secondary text-center" onClick={() => setIsMenuOpen(false)}>
                  Iniciar Sesión
                </Link>
                <Link href="/register" className="btn-primary text-center" onClick={() => setIsMenuOpen(false)}>
                  Registrarse
                </Link>
              </div>
            ) : (
              <div className="flex flex-col space-y-2 pt-2">
                <div className="flex items-center py-2">
                  <div className="w-8 h-8 bg-button rounded-full flex items-center justify-center text-white">
                    <User size={18} />
                  </div>
                  <span className="ml-2 font-medium">{user.name}</span>
                </div>
                <button
                  onClick={() => {
                    logout()
                    setIsMenuOpen(false)
                  }}
                  className="btn-secondary flex items-center justify-center"
                >
                  <LogOut size={18} className="mr-2" />
                  Cerrar Sesión
                </button>
              </div>
            )}
          </nav>
        )}
      </div>
    </header>
  )
}

