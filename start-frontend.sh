#!/bin/bash

# Script de automatización para iniciar el frontend en Linux/Unix
# Autor: DevOps Team
# Fecha: 2024
# Descripción: Este script automatiza la instalación de dependencias y el inicio del servidor de desarrollo

echo -e "\e[36m🚀 Iniciando el proceso de configuración del frontend...\e[0m"

# Navegar al directorio del frontend
cd ./aidguide_04_ws/src/aidguide_04_web

# Verificar si node_modules existe
if [ ! -d "node_modules" ]; then
    echo -e "\e[33m📦 Instalando dependencias...\e[0m"
    npm install --legacy-peer-deps
else
    echo -e "\e[32m✅ Las dependencias ya están instaladas\e[0m"
fi

# Iniciar el servidor de desarrollo
echo -e "\e[36m🌐 Iniciando el servidor de desarrollo...\e[0m"
npm run dev 