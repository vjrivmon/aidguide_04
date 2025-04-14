import { NextRequest, NextResponse } from 'next/server'

// Importar funciones del endpoint de verificación
// En una aplicación real, estas funciones estarían en un módulo compartido
function generateVerificationCode(): string {
  return Math.floor(100000 + Math.random() * 900000).toString()
}

// Base de datos simulada para almacenar los códigos de verificación
// En una aplicación real, esto estaría en una base de datos
const verificationCodesDB = new Map<number, string>()

// Simulación de base de datos de usuarios para obtener el email
const userDB = new Map<number, { email: string }>([
  [1, { email: 'usuario1@ejemplo.com' }],
  [2, { email: 'usuario2@ejemplo.com' }],
  [3, { email: 'usuario3@ejemplo.com' }],
])

// Función para simular el envío de un correo electrónico
async function sendEmailMock(email: string, code: string): Promise<boolean> {
  console.log(`[MOCK] Reenviando correo a ${email} con código ${code}`)
  return true
}

export async function POST(request: NextRequest) {
  try {
    const body = await request.json()
    const { userId } = body

    // Verificar que userId esté presente
    if (!userId) {
      return NextResponse.json(
        { success: false, message: 'ID de usuario faltante' },
        { status: 400 }
      )
    }

    // Buscar el usuario
    const user = userDB.get(userId)
    if (!user) {
      return NextResponse.json(
        { success: false, message: 'Usuario no encontrado' },
        { status: 404 }
      )
    }

    // Generar un nuevo código de verificación
    const code = generateVerificationCode()
    
    // Almacenar el código
    verificationCodesDB.set(userId, code)
    
    // Simular envío de correo
    await sendEmailMock(user.email, code)
    
    return NextResponse.json({
      success: true,
      message: 'Código de verificación reenviado correctamente',
      // NOTA: En producción NUNCA deberíamos devolver el código,
      // esto es solo para propósitos de demostración
      debug: { code }
    })
  } catch (error) {
    console.error('Error al reenviar código de verificación:', error)
    return NextResponse.json(
      { success: false, message: 'Error interno del servidor' },
      { status: 500 }
    )
  }
} 