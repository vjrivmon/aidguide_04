"use client"

import { useState, useEffect } from "react"
import { useRouter } from "next/navigation"
import Link from "next/link"
import { QrCode, ArrowLeft, CheckCircle2 } from "lucide-react"
import dynamic from 'next/dynamic'

// Importación dinámica para evitar errores de SSR
const ReactConfetti = dynamic(() => import('react-confetti'), {
  ssr: false
})

export default function LinkRobot() {
  const router = useRouter()
  const [view, setView] = useState<"code" | "qr">("code")
  const [code, setCode] = useState<string[]>(["", "", "", "", "", ""])
  const [error, setError] = useState("")
  const [success, setSuccess] = useState(false)
  const [loading, setLoading] = useState(false)
  const [showConfetti, setShowConfetti] = useState(false)
  const [confettiOpacity, setConfettiOpacity] = useState(1)
  const [windowSize, setWindowSize] = useState({ width: 0, height: 0 })

  // Referencia para mantener el foco en los inputs del código
  const inputRefs = Array(6).fill(0).map(() => useState<HTMLInputElement | null>(null))

  // Efecto para obtener el tamaño de la ventana para el confeti
  useEffect(() => {
    const handleResize = () => {
      setWindowSize({
        width: window.innerWidth,
        height: window.innerHeight,
      })
    }
    
    // Establecer tamaño inicial
    handleResize()
    
    window.addEventListener('resize', handleResize)
    return () => window.removeEventListener('resize', handleResize)
  }, [])

  const handleCodeChange = (index: number, value: string) => {
    // Validar que solo sean números
    if (value && !/^\d+$/.test(value)) return;

    const newCode = [...code];
    newCode[index] = value;
    setCode(newCode);

    // Si se ingresó un número, mover al siguiente input
    if (value !== "" && index < 5) {
      inputRefs[index + 1][0]?.focus();
    }
  }

  const handleKeyDown = (index: number, e: React.KeyboardEvent<HTMLInputElement>) => {
    // Si se presiona Backspace y está vacío, ir al campo anterior
    if (e.key === "Backspace" && code[index] === "" && index > 0) {
      inputRefs[index - 1][0]?.focus();
    } else if (e.key === "ArrowLeft" && index > 0) {
      inputRefs[index - 1][0]?.focus();
    } else if (e.key === "ArrowRight" && index < 5) {
      inputRefs[index + 1][0]?.focus();
    }
  }

  const handlePaste = (e: React.ClipboardEvent) => {
    e.preventDefault();
    const pastedData = e.clipboardData.getData("text");
    
    // Verificar si los datos pegados son 6 dígitos
    if (/^\d{6}$/.test(pastedData)) {
      const newCode = pastedData.split("");
      setCode(newCode);
    }
  }

  const validateCode = () => {
    setError("");
    
    // Verificar que todos los campos estén completos
    if (code.some(digit => digit === "")) {
      setError("Por favor, introduce el código completo de 6 dígitos.");
      return false;
    }
    
    return true;
  }

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    
    if (!validateCode()) return;
    
    setLoading(true);
    
    // Simulación de verificación del código
    setTimeout(() => {
      const fullCode = code.join("");
      
      // Para demostración, vamos a considerar que "123456" es un código válido
      if (fullCode === "123456") {
        // Guardar en localStorage que el usuario ya ha vinculado un robot
        localStorage.setItem("has-linked-robot", "true");
        
        setSuccess(true);
        
        // Mostrar confeti con opacidad completa
        setConfettiOpacity(1);
        setShowConfetti(true);
        
        // Iniciar el desvanecimiento después de 2 segundos
        setTimeout(() => {
          // Desvanecimiento gradual durante 2 segundos
          const fadeInterval = setInterval(() => {
            setConfettiOpacity((prevOpacity) => {
              const newOpacity = prevOpacity - 0.05;
              if (newOpacity <= 0) {
                clearInterval(fadeInterval);
                setShowConfetti(false);
                return 0;
              }
              return newOpacity;
            });
          }, 100);
        }, 2000);
        
        // Redirigir a la página principal después de 5 segundos
        setTimeout(() => {
          router.push("/welcome");
        }, 5000);
      } else {
        setError("El código introducido no es válido. Por favor, verifica e inténtalo de nuevo.");
      }
      
      setLoading(false);
    }, 1500);
  }

  return (
    <div className="min-h-[calc(100vh-200px)] flex flex-col items-center justify-center py-12 px-4 sm:px-6 lg:px-8 bg-background">
      {/* Componente Confetti con transición de opacidad */}
      {showConfetti && (
        <div className="fixed inset-0 z-50 pointer-events-none transition-opacity duration-300 ease-out"
             style={{ opacity: confettiOpacity }}>
          <ReactConfetti
            width={windowSize.width}
            height={windowSize.height}
            recycle={true}
            numberOfPieces={500}
            gravity={0.15}
          />
        </div>
      )}
      
      <div className="w-full max-w-md space-y-8 bg-white p-8 rounded-33 shadow-md">
        {success ? (
          <div className="text-center space-y-6">
            <div className="flex justify-center">
              <CheckCircle2 className="h-16 w-16 text-green-500" />
            </div>
            <h2 className="text-2xl font-bold text-gray-900">¡Robot vinculado correctamente!</h2>
            <p className="text-gray-600">
              Tu robot ha sido vinculado con éxito a tu cuenta. 
              Serás redirigido automáticamente a la página principal en unos segundos.
            </p>
            <button
              onClick={() => router.push("/welcome")}
              className="mt-6 w-full btn-primary"
            >
              Continuar
            </button>
          </div>
        ) : (
          <>
            <div>
              <h1 className="text-center text-2xl font-bold">Vincular tu robot</h1>
              <p className="mt-2 text-center text-sm text-gray-600">
                Introduce el código de 6 dígitos o escanea el QR proporcionado con tu robot
              </p>
            </div>

            <div className="flex justify-center space-x-4 my-6">
              <button
                type="button"
                onClick={() => setView("code")}
                className={`px-4 py-2 rounded-md ${
                  view === "code"
                    ? "bg-button text-white"
                    : "bg-gray-100 text-gray-700 hover:bg-gray-200"
                }`}
              >
                Código
              </button>
              <button
                type="button"
                onClick={() => setView("qr")}
                className={`px-4 py-2 rounded-md ${
                  view === "qr"
                    ? "bg-button text-white"
                    : "bg-gray-100 text-gray-700 hover:bg-gray-200"
                }`}
              >
                QR
              </button>
            </div>

            {error && (
              <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded relative" role="alert">
                <span className="block">{error}</span>
              </div>
            )}

            {view === "code" ? (
              <form onSubmit={handleSubmit} className="mt-6 space-y-6">
                <div>
                  <label htmlFor="code-input" className="sr-only">
                    Código de vinculación
                  </label>
                  <div 
                    className="flex justify-between gap-2"
                    onPaste={handlePaste}
                  >
                    {code.map((digit, index) => (
                      <input
                        key={index}
                        ref={(el) => inputRefs[index][1](el)}
                        type="text"
                        maxLength={1}
                        className="form-input w-12 h-12 text-center text-xl font-bold"
                        value={digit}
                        onChange={(e) => handleCodeChange(index, e.target.value)}
                        onKeyDown={(e) => handleKeyDown(index, e)}
                        required
                      />
                    ))}
                  </div>
                </div>

                <div>
                  <button
                    type="submit"
                    className="btn-primary w-full flex justify-center items-center"
                    disabled={loading}
                  >
                    {loading ? (
                      <span className="flex items-center">
                        <svg
                          className="animate-spin -ml-1 mr-3 h-5 w-5 text-white"
                          xmlns="http://www.w3.org/2000/svg"
                          fill="none"
                          viewBox="0 0 24 24"
                        >
                          <circle
                            className="opacity-25"
                            cx="12"
                            cy="12"
                            r="10"
                            stroke="currentColor"
                            strokeWidth="4"
                          ></circle>
                          <path
                            className="opacity-75"
                            fill="currentColor"
                            d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                          ></path>
                        </svg>
                        Verificando...
                      </span>
                    ) : (
                      "Vincular robot"
                    )}
                  </button>
                </div>

                <div className="text-center mt-4">
                  <button
                    type="button"
                    className="text-button text-sm font-medium hover:underline"
                    onClick={() => {
                      // En una aplicación real, aquí se implementaría la lógica para reenviar el código
                      alert("Se ha enviado un nuevo código a tu dispositivo");
                    }}
                  >
                    ¿No recibiste tu código?
                  </button>
                </div>
              </form>
            ) : (
              <div className="mt-6 text-center space-y-6">
                <div className="flex justify-center">
                  <div className="border-4 border-gray-300 rounded-lg p-4 inline-block">
                    <QrCode className="h-48 w-48 text-gray-800" />
                    <p className="mt-2 text-sm text-gray-500">Escanea el QR de tu robot</p>
                  </div>
                </div>
                <p className="text-gray-600">
                  Coloca el código QR frente a la cámara para vincular tu robot automáticamente.
                </p>
              </div>
            )}

            <div className="mt-6">
              <Link href="/" className="flex items-center text-button hover:underline">
                <ArrowLeft className="mr-2 h-4 w-4" />
                Volver al inicio
              </Link>
            </div>
          </>
        )}
      </div>
    </div>
  )
} 