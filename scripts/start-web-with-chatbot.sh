#!/bin/bash

# Colores para los mensajes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${YELLOW}🚀 AIDGUIDE 04 - WEB CON CHATBOT OLLAMA ${CYAN}              ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Verificar si estamos en el directorio correcto
if [ ! -d "./aidguide_04_ws" ]; then
    echo -e "${RED}❌ No se encuentra el directorio del workspace${NC}"
    echo -e "${YELLOW}📝 Asegúrate de estar en el directorio raíz del proyecto${NC}"
    exit 1
fi

# Iniciar el servicio Ollama en segundo plano
echo -e "${YELLOW}📝 Iniciando el servicio Ollama...${NC}"
./start-ollama &
OLLAMA_PID=$!

# Esperar a que Ollama esté listo
echo -e "${YELLOW}📝 Esperando a que el servicio Ollama esté disponible...${NC}"
MAX_ATTEMPTS=30
ATTEMPT=0
OLLAMA_READY=false

while [ $ATTEMPT -lt $MAX_ATTEMPTS ]; do
    if curl -s http://localhost:11434/api/health &> /dev/null; then
        OLLAMA_READY=true
        break
    fi
    ATTEMPT=$((ATTEMPT+1))
    echo -e "${YELLOW}⏳ Esperando a Ollama... Intento $ATTEMPT de $MAX_ATTEMPTS${NC}"
    sleep 1
done

if [ "$OLLAMA_READY" = false ]; then
    echo -e "${RED}❌ No se pudo iniciar el servicio Ollama después de $MAX_ATTEMPTS intentos${NC}"
    echo -e "${YELLOW}📝 Por favor, inicia el servicio manualmente con ./start-ollama${NC}"
    kill $OLLAMA_PID 2>/dev/null
else
    echo -e "${GREEN}✅ Servicio Ollama iniciado correctamente${NC}"
fi

# Iniciar la aplicación web
echo -e "${YELLOW}📝 Iniciando la aplicación web...${NC}"
WORKSPACE_PATH="$(pwd)/aidguide_04_ws"
cd "$WORKSPACE_PATH/src/aidguide_04_web" && npm run dev &
WEB_PID=$!

echo -e "${GREEN}✅ Aplicación web iniciada${NC}"
echo -e "${CYAN}🌐 La aplicación web está disponible en: http://localhost:3000${NC}"
echo -e "${YELLOW}💡 El chatbot está integrado en la web y listo para usar${NC}"
echo -e "${YELLOW}⚠️ Este terminal debe permanecer abierto para que los servicios funcionen${NC}"
echo -e "${YELLOW}⚠️ Presione Ctrl+C para detener todos los servicios${NC}"

# Función para manejar la señal de interrupción
function cleanup() {
    echo -e "${YELLOW}📝 Deteniendo servicios...${NC}"
    kill $OLLAMA_PID 2>/dev/null
    kill $WEB_PID 2>/dev/null
    echo -e "${GREEN}✅ Servicios detenidos${NC}"
    exit 0
}

# Configurar la captura de señal de interrupción
trap cleanup INT

# Mantener el script en ejecución
wait 