import { NextRequest, NextResponse } from 'next/server'

// Base de datos simulada para almacenar los códigos de verificación
const verificationCodesDB = new Map<number, string>()

// Función para generar un código aleatorio de 6 dígitos
function generateVerificationCode(): string {
  return Math.floor(100000 + Math.random() * 900000).toString()
}

// Función para simular el envío de un correo electrónico
async function sendEmailMock(email: string, code: string): Promise<boolean> {
  console.log(`[MOCK] Enviando correo a ${email} con código ${code}`)
  return true
}

export async function POST(request: NextRequest) {
  try {
    const body = await request.json()
    const { userId, code } = body

    // Verificar que userId y code estén presentes
    if (!userId || !code) {
      return NextResponse.json(
        { success: false, message: 'Usuario o código faltantes' },
        { status: 400 }
      )
    }

    // Obtener el código almacenado
    const storedCode = verificationCodesDB.get(userId)

    // Si no hay código almacenado, generar uno nuevo
    if (!storedCode) {
      // En un entorno real, deberíamos verificar que el usuario existe
      // y posiblemente generar un nuevo código
      return NextResponse.json(
        { success: false, message: 'Código no encontrado. Solicite un nuevo código.' },
        { status: 400 }
      )
    }

    // Verificar que el código coincida
    if (code === storedCode) {
      // Eliminar el código verificado
      verificationCodesDB.delete(userId)
      
      return NextResponse.json({
        success: true,
        message: 'Correo electrónico verificado correctamente'
      })
    } else {
      return NextResponse.json(
        { success: false, message: 'Código inválido' },
        { status: 400 }
      )
    }
  } catch (error) {
    console.error('Error en la verificación de correo:', error)
    return NextResponse.json(
      { success: false, message: 'Error interno del servidor' },
      { status: 500 }
    )
  }
}

// Endpoint para generar un nuevo código de verificación
export async function GET(request: NextRequest) {
  const url = new URL(request.url)
  const userId = url.searchParams.get('userId')
  const email = url.searchParams.get('email')

  if (!userId || !email) {
    return NextResponse.json(
      { success: false, message: 'Usuario o email faltantes' },
      { status: 400 }
    )
  }

  // Generar un código de verificación
  const code = generateVerificationCode()
  
  // Almacenar el código
  verificationCodesDB.set(Number(userId), code)
  
  // Simular envío de correo
  await sendEmailMock(email, code)
  
  return NextResponse.json({
    success: true,
    message: 'Código de verificación enviado correctamente',
    // NOTA: En producción NUNCA deberíamos devolver el código,
    // esto es solo para propósitos de demostración
    debug: { code }
  })
} 