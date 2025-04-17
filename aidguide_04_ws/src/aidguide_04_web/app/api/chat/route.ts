import { NextRequest, NextResponse } from 'next/server';

export async function POST(request: NextRequest) {
  try {
    // Extraer el cuerpo de la solicitud
    const body = await request.json();
    const { messages } = body;

    if (!messages || !Array.isArray(messages)) {
      return NextResponse.json(
        { error: 'El campo "messages" es requerido y debe ser un array' },
        { status: 400 }
      );
    }

    // Comunicarse con la API de Ollama
    const ollamaResponse = await fetch('http://localhost:11434/api/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        model: 'mistral',
        messages,
        stream: false,
      }),
    });

    if (!ollamaResponse.ok) {
      const errorData = await ollamaResponse.text();
      console.error('Error del servidor Ollama:', errorData);
      
      return NextResponse.json(
        { error: 'Error al comunicarse con el servicio de Ollama' },
        { status: 502 }
      );
    }

    // Devolver la respuesta de Ollama
    const data = await ollamaResponse.json();
    return NextResponse.json(data);
  } catch (error) {
    console.error('Error al procesar la solicitud:', error);
    
    return NextResponse.json(
      { error: 'Error interno del servidor' },
      { status: 500 }
    );
  }
}

// Ruta para verificar la disponibilidad del servicio de Ollama
export async function GET() {
  try {
    const response = await fetch('http://localhost:11434/', {
      method: 'GET',
    });

    if (response.ok) {
      return NextResponse.json({ status: 'available' });
    } else {
      return NextResponse.json(
        { status: 'unavailable', error: 'El servicio de Ollama no está disponible' },
        { status: 503 }
      );
    }
  } catch (error) {
    console.error('Error al verificar disponibilidad:', error);
    
    return NextResponse.json(
      { status: 'unavailable', error: 'No se puede conectar con el servicio de Ollama' },
      { status: 503 }
    );
  }
} 