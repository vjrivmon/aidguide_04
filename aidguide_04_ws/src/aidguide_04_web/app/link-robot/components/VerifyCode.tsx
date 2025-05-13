"use client"

import React, { useState, useRef, useEffect } from 'react'

interface VerifyCodeProps {
  onComplete: (code: string) => void
  disabled?: boolean
  className?: string
}

export default function VerifyCode({ onComplete, disabled = false, className = '' }: VerifyCodeProps) {
  const [code, setCode] = useState<string[]>(Array(6).fill(''))
  const inputsRef = useRef<(HTMLInputElement | null)[]>([])
  
  // Enfoque en el primer input cuando el componente se carga
  useEffect(() => {
    if (inputsRef.current[0] && !disabled) {
      inputsRef.current[0].focus()
    }
  }, [disabled])
  
  // Llamar a onComplete cuando se completen los 6 dígitos
  useEffect(() => {
    if (code.every(digit => digit !== '') && !disabled) {
      onComplete(code.join(''))
    }
  }, [code, onComplete, disabled])
  
  const handleChange = (index: number, value: string) => {
    // Solo permitir números
    if (!/^\d*$/.test(value)) return
    
    const newCode = [...code]
    newCode[index] = value
    setCode(newCode)
    
    // Avanzar automáticamente al siguiente campo si se ingresa un número
    if (value.length === 1 && index < 5 && inputsRef.current[index + 1]) {
      inputsRef.current[index + 1]?.focus()
    }
  }
  
  const handleKeyDown = (index: number, e: React.KeyboardEvent<HTMLInputElement>) => {
    // Retroceder con backspace si el campo está vacío
    if (e.key === 'Backspace' && index > 0 && code[index] === '') {
      inputsRef.current[index - 1]?.focus()
    }
  }
  
  const handlePaste = (e: React.ClipboardEvent<HTMLInputElement>) => {
    e.preventDefault()
    const pastedCode = e.clipboardData.getData('text').trim()
    
    // Verificar si es un código de 6 dígitos
    if (/^\d{6}$/.test(pastedCode)) {
      const digits = pastedCode.split('')
      setCode(digits)
      
      // Enfoque en el último campo
      inputsRef.current[5]?.focus()
    }
  }
  
  return (
    <div className={`flex items-center justify-center gap-2 ${className}`}>
      {code.map((digit, index) => (
        <input
          key={index}
          ref={el => inputsRef.current[index] = el}
          type="text"
          inputMode="numeric"
          pattern="[0-9]*"
          maxLength={1}
          value={digit}
          onChange={(e) => handleChange(index, e.target.value)}
          onKeyDown={(e) => handleKeyDown(index, e)}
          onPaste={index === 0 ? handlePaste : undefined}
          disabled={disabled}
          className="w-12 h-12 text-center text-xl font-semibold border-2 border-gray-300 rounded-md focus:border-blue-500 focus:ring-blue-500 focus:outline-none disabled:bg-gray-100 disabled:cursor-not-allowed"
          aria-label={`Dígito ${index + 1} del código`}
        />
      ))}
    </div>
  )
} 