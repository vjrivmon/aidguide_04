# Script de automatización para iniciar el frontend en Windows
# Autor: DevOps Team
# Fecha: 2024
# Descripción: Este script automatiza la instalación de dependencias y el inicio del servidor de desarrollo

Write-Host "🚀 Iniciando el proceso de configuración del frontend..." -ForegroundColor Cyan

# Navegar al directorio del frontend
Set-Location -Path ".\aidguide_04_ws\src\aidguide_04_web"

# Verificar si node_modules existe
if (-not (Test-Path -Path ".\node_modules")) {
    Write-Host "📦 Instalando dependencias..." -ForegroundColor Yellow
    npm install --legacy-peer-deps
} else {
    Write-Host "✅ Las dependencias ya están instaladas" -ForegroundColor Green
}

# Iniciar el servidor de desarrollo
Write-Host "🌐 Iniciando el servidor de desarrollo..." -ForegroundColor Cyan
npm run dev 