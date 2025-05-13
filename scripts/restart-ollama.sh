#!/bin/bash

# Colores para los mensajes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${CYAN}╔════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}║  ${YELLOW}🔄 AIDGUIDE 04 - REINICIO DEL SERVICIO OLLAMA ${CYAN}         ║${NC}"
echo -e "${CYAN}║                                                            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Detener cualquier proceso de Ollama existente
echo -e "${YELLOW}📝 Buscando procesos Ollama en ejecución...${NC}"
OLLAMA_PIDS=$(ps aux | grep ollama | grep -v grep | awk '{print $2}')

if [ -n "$OLLAMA_PIDS" ]; then
    echo -e "${YELLOW}📝 Deteniendo procesos Ollama existentes...${NC}"
    for PID in $OLLAMA_PIDS; do
        echo -e "${YELLOW}  - Deteniendo proceso con PID $PID...${NC}"
        kill -9 $PID 2>/dev/null
    done
    echo -e "${GREEN}✅ Procesos Ollama detenidos correctamente${NC}"
else
    echo -e "${GREEN}✅ No se encontraron procesos Ollama en ejecución${NC}"
fi

# Verificar si hay algún proceso usando el puerto 11434
echo -e "${YELLOW}📝 Verificando si hay procesos usando el puerto 11434...${NC}"
PORT_PID=$(lsof -t -i:11434 2>/dev/null)

if [ -n "$PORT_PID" ]; then
    echo -e "${YELLOW}📝 Deteniendo proceso que usa el puerto 11434 (PID: $PORT_PID)...${NC}"
    kill -9 $PORT_PID 2>/dev/null
    echo -e "${GREEN}✅ Proceso detenido correctamente${NC}"
else
    echo -e "${GREEN}✅ No hay procesos usando el puerto 11434${NC}"
fi

# Esperar a que los procesos terminen
echo -e "${YELLOW}📝 Esperando a que los procesos terminen completamente...${NC}"
sleep 3

# Iniciar Ollama con el script original
echo -e "${YELLOW}📝 Iniciando el servicio Ollama...${NC}"
./start-ollama &
OLLAMA_PID=$!

# Esperar a que Ollama esté listo
echo -e "${YELLOW}📝 Esperando a que el servicio esté disponible...${NC}"
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
    echo -e "${YELLOW}📝 Verifica la instalación de Ollama o reinicia el sistema${NC}"
    kill $OLLAMA_PID 2>/dev/null
    exit 1
else
    echo -e "${GREEN}✅ Servicio Ollama reiniciado correctamente${NC}"
    echo -e "${CYAN}🌐 API de Ollama disponible en: http://localhost:11434${NC}"
    echo -e "${YELLOW}💡 Ahora puedes usar el chatbot desde la aplicación web${NC}"
    echo -e "${YELLOW}⚠️ Este terminal debe permanecer abierto para que el servicio funcione${NC}"
    echo -e "${YELLOW}⚠️ Presiona Ctrl+C para detener el servicio${NC}"
fi

# Mantener el script en ejecución
wait $OLLAMA_PID 