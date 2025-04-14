"use client"

import { useState, useEffect } from 'react'
import { useRouter } from 'next/navigation'
import { useAuth } from '@/context/auth-context'
import { toast } from 'sonner'
import Link from 'next/link'
import VerifyCode from '@/app/link-robot/components/VerifyCode'

export default function VerifyEmailPage() {
  const [loading, setLoading] = useState(false)
  const [resendDisabled, setResendDisabled] = useState(false)
  const [countdown, setCountdown] = useState(0)
  const { user, verifyCode, resendVerificationCode, checkVerification } = useAuth()
  const router = useRouter()

  // Redirigir si el usuario no está autenticado o ya está verificado
  useEffect(() => {
    if (!user) {
      router.push('/login')
    } else if (checkVerification()) {
      // Si ya está verificado, redirigir al perfil
      router.push('/profile')
    }
  }, [user, router, checkVerification])

  // Manejar el contador para reenviar código
  useEffect(() => {
    if (countdown > 0) {
      const timer = setTimeout(() => setCountdown(countdown - 1), 1000)
      return () => clearTimeout(timer)
    } else if (countdown === 0 && resendDisabled) {
      setResendDisabled(false)
    }
  }, [countdown, resendDisabled])

  // Enviar código para verificación
  const handleVerifyCode = async (verificationCode: string) => {
    if (!user) {
      toast.error('Debe iniciar sesión para verificar su cuenta')
      return
    }
    
    setLoading(true)
    
    try {
      const success = await verifyCode(verificationCode)
      
      if (success) {
        router.push('/profile')
      } else {
        toast.error('El código ingresado no es válido')
      }
    } catch (error) {
      console.error(error)
    } finally {
      setLoading(false)
    }
  }

  // Reenviar código
  const handleResendCode = async () => {
    if (!user) return
    
    setResendDisabled(true)
    setCountdown(60) // 60 segundos de espera
    
    try {
      await resendVerificationCode()
    } catch (error) {
      console.error(error)
      setResendDisabled(false)
      setCountdown(0)
    }
  }

  if (!user) {
    return <div className="flex items-center justify-center min-h-screen">Redirigiendo...</div>
  }

  return (
    <div className="flex flex-col items-center justify-center min-h-screen bg-gray-50 px-4">
      <div className="w-full max-w-md p-8 space-y-8 bg-white rounded-xl shadow-md">
        <div className="text-center">
          <h1 className="text-2xl font-bold text-gray-900">Verificación de Correo</h1>
          <p className="mt-2 text-sm text-gray-600">
            Hemos enviado un código de verificación a tu correo electrónico corporativo: <strong>{user.email}</strong>
          </p>
        </div>

        <div className="mt-8 space-y-6">
          <div className="flex flex-col items-center">
            <label className="block text-sm font-medium text-gray-700 mb-3">
              Ingresa el código de 6 dígitos
            </label>
            
            <VerifyCode 
              onComplete={handleVerifyCode} 
              disabled={loading}
              className="mb-4" 
            />
          </div>

          <div className="flex flex-col items-center">
            <div className="mt-4 text-center">
              <button
                type="button"
                onClick={handleResendCode}
                disabled={resendDisabled}
                className="text-sm text-blue-600 hover:text-blue-800 disabled:text-gray-400 disabled:cursor-not-allowed transition-colors"
              >
                {resendDisabled 
                  ? `Reenviar código en ${countdown}s` 
                  : "¿No recibiste el código? Reenviar"}
              </button>
            </div>
          </div>
        </div>

        <div className="mt-4 text-center">
          <Link 
            href="/login" 
            className="text-sm text-blue-600 hover:text-blue-800"
          >
            Volver al inicio de sesión
          </Link>
        </div>
      </div>
    </div>
  )
} 