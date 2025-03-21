"use client"

import { useState } from 'react'
import { FaEnvelope, FaKey, FaMobile, FaFingerprint, FaShieldAlt } from 'react-icons/fa'
import Link from 'next/link'

export default function ResetPassword() {
  const [step, setStep] = useState(1)
  const [method, setMethod] = useState('')
  const [email, setEmail] = useState('')
  const [code, setCode] = useState('')
  const [newPassword, setNewPassword] = useState('')
  const [confirmPassword, setConfirmPassword] = useState('')
  const [status, setStatus] = useState({ type: '', message: '' })

  const handleMethodSelect = (selectedMethod: string) => {
    setMethod(selectedMethod)
    setStep(2)
    // Simulamos envío de código de verificación
    setStatus({
      type: 'info',
      message: selectedMethod === 'email' 
        ? 'Se ha enviado un código de verificación a tu correo electrónico.'
        : selectedMethod === 'sms'
        ? 'Se ha enviado un código de verificación a tu teléfono móvil.'
        : 'Por favor, verifica tu identidad usando el método seleccionado.'
    })
  }

  const handleVerifyCode = (e: React.FormEvent) => {
    e.preventDefault()
    // Aquí iría la verificación real del código
    setStep(3)
    setStatus({ type: 'success', message: 'Código verificado correctamente.' })
  }

  const handleResetPassword = async (e: React.FormEvent) => {
    e.preventDefault()
    
    if (newPassword !== confirmPassword) {
      setStatus({ type: 'error', message: 'Las contraseñas no coinciden.' })
      return
    }

    // Aquí iría la lógica real de cambio de contraseña
    setStatus({ 
      type: 'success', 
      message: 'Contraseña actualizada correctamente. Redirigiendo al inicio de sesión...' 
    })
    
    // Simulamos redirección después de 2 segundos
    setTimeout(() => {
      window.location.href = '/login'
    }, 2000)
  }

  return (
    <div className="min-h-screen py-12 px-4 sm:px-6 lg:px-8 bg-background">
      <div className="max-w-md mx-auto">
        <div className="text-center mb-8">
          <h1 className="text-3xl font-bold text-gray-900">Restablecer Contraseña</h1>
          <p className="mt-2 text-gray-600">
            Selecciona un método seguro para verificar tu identidad
          </p>
        </div>

        <div className="bg-white rounded-33 shadow-md p-8">
          {step === 1 && (
            <div className="space-y-6">
              <div className="text-center mb-6">
                <h2 className="text-xl font-semibold text-gray-900">Método de Verificación</h2>
                <p className="mt-1 text-sm text-gray-500">
                  Elige cómo quieres verificar tu identidad
                </p>
              </div>

              <button
                onClick={() => handleMethodSelect('email')}
                className="w-full flex items-center justify-between p-4 border rounded-lg hover:border-button transition-colors"
              >
                <div className="flex items-center">
                  <FaEnvelope className="text-button mr-3" size={24} />
                  <div className="text-left">
                    <div className="font-medium">Correo Electrónico</div>
                    <div className="text-sm text-gray-500">Recibe un código por email</div>
                  </div>
                </div>
              </button>

              <button
                onClick={() => handleMethodSelect('sms')}
                className="w-full flex items-center justify-between p-4 border rounded-lg hover:border-button transition-colors"
              >
                <div className="flex items-center">
                  <FaMobile className="text-button mr-3" size={24} />
                  <div className="text-left">
                    <div className="font-medium">SMS</div>
                    <div className="text-sm text-gray-500">Recibe un código por SMS</div>
                  </div>
                </div>
              </button>

              {/* Solo mostrar si el dispositivo soporta biometría */}
              <button
                onClick={() => handleMethodSelect('biometric')}
                className="w-full flex items-center justify-between p-4 border rounded-lg hover:border-button transition-colors"
              >
                <div className="flex items-center">
                  <FaFingerprint className="text-button mr-3" size={24} />
                  <div className="text-left">
                    <div className="font-medium">Huella Digital</div>
                    <div className="text-sm text-gray-500">Usa tu huella digital</div>
                  </div>
                </div>
              </button>
            </div>
          )}

          {step === 2 && (
            <form onSubmit={handleVerifyCode} className="space-y-6">
              <div className="text-center mb-6">
                <h2 className="text-xl font-semibold text-gray-900">Verificación</h2>
                <p className="mt-1 text-sm text-gray-500">
                  Introduce el código de verificación enviado
                </p>
              </div>

              <div>
                <label htmlFor="code" className="block text-sm font-medium text-gray-700 mb-2">
                  Código de Verificación
                </label>
                <div className="relative">
                  <FaShieldAlt className="absolute left-3 top-3 text-gray-400" size={20} />
                  <input
                    type="text"
                    id="code"
                    value={code}
                    onChange={(e) => setCode(e.target.value)}
                    className="pl-10 w-full px-4 py-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-button focus:border-transparent"
                    placeholder="Introduce el código"
                    required
                  />
                </div>
              </div>

              <button
                type="submit"
                className="w-full bg-button text-white py-3 px-6 rounded-md hover:bg-opacity-90 transition-colors font-medium"
              >
                Verificar Código
              </button>
            </form>
          )}

          {step === 3 && (
            <form onSubmit={handleResetPassword} className="space-y-6">
              <div className="text-center mb-6">
                <h2 className="text-xl font-semibold text-gray-900">Nueva Contraseña</h2>
                <p className="mt-1 text-sm text-gray-500">
                  Crea una contraseña segura
                </p>
              </div>

              <div>
                <label htmlFor="newPassword" className="block text-sm font-medium text-gray-700 mb-2">
                  Nueva Contraseña
                </label>
                <div className="relative">
                  <FaKey className="absolute left-3 top-3 text-gray-400" size={20} />
                  <input
                    type="password"
                    id="newPassword"
                    value={newPassword}
                    onChange={(e) => setNewPassword(e.target.value)}
                    className="pl-10 w-full px-4 py-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-button focus:border-transparent"
                    placeholder="Mínimo 8 caracteres"
                    required
                    minLength={8}
                  />
                </div>
              </div>

              <div>
                <label htmlFor="confirmPassword" className="block text-sm font-medium text-gray-700 mb-2">
                  Confirmar Contraseña
                </label>
                <div className="relative">
                  <FaKey className="absolute left-3 top-3 text-gray-400" size={20} />
                  <input
                    type="password"
                    id="confirmPassword"
                    value={confirmPassword}
                    onChange={(e) => setConfirmPassword(e.target.value)}
                    className="pl-10 w-full px-4 py-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-button focus:border-transparent"
                    placeholder="Repite la contraseña"
                    required
                    minLength={8}
                  />
                </div>
              </div>

              <button
                type="submit"
                className="w-full bg-button text-white py-3 px-6 rounded-md hover:bg-opacity-90 transition-colors font-medium"
              >
                Actualizar Contraseña
              </button>
            </form>
          )}

          {status.message && (
            <div className={`mt-4 p-4 rounded-md ${
              status.type === 'success' ? 'bg-green-100 text-green-700' :
              status.type === 'error' ? 'bg-red-100 text-red-700' :
              'bg-blue-100 text-blue-700'
            }`}>
              {status.message}
            </div>
          )}

          <div className="mt-6 text-center">
            <Link 
              href="/login"
              className="text-button hover:text-opacity-80 transition-colors"
            >
              Volver al inicio de sesión
            </Link>
          </div>
        </div>
      </div>
    </div>
  )
} 