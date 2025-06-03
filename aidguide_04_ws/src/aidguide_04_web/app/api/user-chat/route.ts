import { NextRequest, NextResponse } from 'next/server';

export async function POST(request: NextRequest) {
  try {
    const body = await request.json();
    const { messages } = body;

    if (!messages || !Array.isArray(messages)) {
      return NextResponse.json(
        { error: 'El campo "messages" es requerido y debe ser un array' },
        { status: 400 }
      );
    }

    const ollamaResponse = await fetch('http://localhost:11434/api/chat', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        model: 'mistral',
        messages, // Esto incluirá el system_prompt específico del UserChatbotProvider
        stream: false,
      }),
    });

    if (!ollamaResponse.ok) {
      const errorData = await ollamaResponse.text();
      console.error('Error del servidor Ollama (en /api/user-chat):', errorData);
      return NextResponse.json(
        { error: 'Error al comunicarse con el servicio de Ollama' },
        { status: ollamaResponse.status } // Devolver el mismo estado que Ollama
      );
    }

    const data = await ollamaResponse.json();
    return NextResponse.json(data);
  } catch (error) {
    console.error('Error al procesar la solicitud en /api/user-chat:', error);
    return NextResponse.json(
      { error: 'Error interno del servidor en /api/user-chat' },
      { status: 500 }
    );
  }
}

export async function GET() {
  try {
    // Esta ruta GET puede usarse para verificar la disponibilidad específica de este endpoint si es necesario
    // o simplemente reusar la lógica de la API de Ollama general.
    const response = await fetch('http://localhost:11434/', { // Ping a la raíz de Ollama para ver si está arriba
      method: 'GET',
    });

    if (response.ok) {
      return NextResponse.json({ status: 'available' });
    } else {
      return NextResponse.json(
        { status: 'unavailable', error: 'El servicio de Ollama no está disponible (verificado desde /api/user-chat)' },
        { status: 503 }
      );
    }
  } catch (error) {
    console.error('Error al verificar disponibilidad (en /api/user-chat):', error);
    return NextResponse.json(
      { status: 'unavailable', error: 'No se puede conectar con el servicio de Ollama (verificado desde /api/user-chat)' },
      { status: 503 }
    );
  }
} 