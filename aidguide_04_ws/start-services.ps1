# Script para iniciar todos los servicios de AidGuide (Frontend y Backend)
# Autor: Vicente
# Fecha: 2024

# Variables de configuración
$SCRIPT_DIR = Split-Path -Parent $MyInvocation.MyCommand.Path
$BACKEND_PATH = Join-Path $SCRIPT_DIR "src\aidguide_04_backend"
$FRONTEND_PATH = Join-Path $SCRIPT_DIR "src\aidguide_04_web"
$PROJECT_NAME = "aidguide_04"
$MAX_RETRIES = 2

# Iconos para mejorar la visualización
$ICON_CHECK = "✅"
$ICON_ERROR = "❌"
$ICON_WARNING = "⚠️"
$ICON_INFO = "ℹ️"
$ICON_FOLDER = "📂"
$ICON_ROCKET = "🚀"
$ICON_CLEANING = "🧹"
$ICON_DOCKER = "🐳"
$ICON_LOADING = "⏳"
$ICON_RETRY = "🔄"
$ICON_WEB = "🌐"
$ICON_DATABASE = "🗄️"
$ICON_DOCS = "📚"
$ICON_API = "🔌"

# Función para mostrar una barra de progreso
function Show-Progress {
    param (
        [int]$Duration,
        [string]$Message
    )
    
    $width = 50
    $barChar = "▓"
    
    Write-Host "$Message" -ForegroundColor Yellow
    for ($i = 0; $i -lt $width; $i++) {
        Write-Host $barChar -NoNewline -ForegroundColor Cyan
        Start-Sleep -Milliseconds ($Duration * 1000 / $width)
    }
    Write-Host " ¡Completado!" -ForegroundColor Green
}

# Función para mostrar el encabezado
function Show-Header {
    Clear-Host
    Write-Host ""
    Write-Host "╔════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
    Write-Host "║                 $ICON_ROCKET AIDGUIDE LAUNCHER $ICON_ROCKET                    ║" -ForegroundColor Cyan
    Write-Host "╚════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Iniciando servicios para el proyecto AidGuide..." -ForegroundColor Blue
    Write-Host ""
}

# Función para verificar si un directorio existe
function Verify-Directory {
    param (
        [string]$Path,
        [string]$Name
    )
    Write-Host "$ICON_INFO  Verificando directorio $Name... " -ForegroundColor Yellow -NoNewline
    if (-Not (Test-Path -Path $Path)) {
        Write-Host "$ICON_ERROR No encontrado" -ForegroundColor Red
        Write-Host "Directorio actual: $(Get-Location)" -ForegroundColor Yellow
        Write-Host "Directorios disponibles:" -ForegroundColor Yellow
        Get-ChildItem -Path $SCRIPT_DIR -Directory | Select-Object Name
        return $false
    }
    Write-Host "$ICON_CHECK Encontrado" -ForegroundColor Green
    return $true
}

# Mostrar el encabezado
Show-Header

# Verificar si los directorios existen
Write-Host "▶ VERIFICANDO DIRECTORIOS" -ForegroundColor Cyan
if (-Not (Verify-Directory -Path $BACKEND_PATH -Name "Backend")) {
    exit 1
}

# Verificar el directorio del frontend
Verify-Directory -Path $FRONTEND_PATH -Name "Frontend" | Out-Null
Write-Host ""

# Navegamos al directorio del backend donde está el docker-compose.yml
Write-Host "▶ PREPARANDO ENTORNO" -ForegroundColor Cyan
Write-Host "$ICON_FOLDER Navegando a $BACKEND_PATH" -ForegroundColor Yellow
Set-Location -Path $BACKEND_PATH

# Comprobamos si Docker está en ejecución
Write-Host "$ICON_INFO  Comprobando estado de Docker... " -ForegroundColor Yellow -NoNewline
try {
    docker info | Out-Null
    Write-Host "$ICON_CHECK En ejecución" -ForegroundColor Green
} catch {
    Write-Host "$ICON_ERROR No en ejecución" -ForegroundColor Red
    Write-Host "Docker no está en ejecución. Por favor, inicie Docker Desktop e intente nuevamente." -ForegroundColor Red
    exit 1
}

# Verificar si existe el archivo docker-compose.yml
Write-Host "$ICON_INFO  Verificando archivo docker-compose.yml... " -ForegroundColor Yellow -NoNewline
if (-Not (Test-Path -Path "docker-compose.yml")) {
    Write-Host "$ICON_ERROR No encontrado" -ForegroundColor Red
    Write-Host "Error: No se encontró el archivo docker-compose.yml en $(Get-Location)" -ForegroundColor Red
    exit 1
} else {
    Write-Host "$ICON_CHECK Encontrado" -ForegroundColor Green
}
Write-Host ""

# Limpiar todos los contenedores relacionados con el proyecto
Write-Host "▶ PREPARANDO CONTENEDORES" -ForegroundColor Cyan
Write-Host "$ICON_CLEANING Limpiando contenedores existentes..." -ForegroundColor Yellow
docker-compose down | Out-Null
$containers = docker ps -a --filter "name=aidguide" --format "{{.Names}}"
if ($containers) {
    Write-Host "  $ICON_INFO  Eliminando contenedores adicionales..." -ForegroundColor Yellow
    docker rm -f $containers | Out-Null
}
Show-Progress -Duration 1 -Message "$ICON_LOADING Limpieza en progreso..."
Write-Host ""

# Función para iniciar los contenedores y manejar reintentos
function Start-Containers {
    param (
        [int]$Retry = 0
    )
    
    # Iniciamos los contenedores con docker-compose
    Write-Host "▶ INICIANDO SERVICIOS (Intento $($Retry+1)/$($MAX_RETRIES+1))" -ForegroundColor Cyan
    Write-Host "$ICON_DOCKER Creando y arrancando contenedores..." -ForegroundColor Yellow
    docker-compose up -d --build
    
    # Mostramos una barra de progreso mientras los contenedores se inician
    Show-Progress -Duration 3 -Message "$ICON_LOADING Inicializando servicios..."
    
    # Verificamos el estado de los contenedores
    Write-Host ""
    Write-Host "$ICON_INFO  ESTADO DE LOS SERVICIOS:" -ForegroundColor Magenta
    Write-Host "╭───────────────────────────────────────────────────────────╮" -ForegroundColor Cyan
    docker-compose ps
    Write-Host "╰───────────────────────────────────────────────────────────╯" -ForegroundColor Cyan
    
    # Verificar si los servicios están en ejecución
    $apiRunning = docker ps --filter "name=aidguide_api" --format "{{.Names}}" | Select-String -Pattern "aidguide_api"
    $frontendRunning = docker ps --filter "name=aidguide_frontend" --format "{{.Names}}" | Select-String -Pattern "aidguide_frontend"
    
    if (-Not $apiRunning) {
        Write-Host ""
        Write-Host "$ICON_WARNING El contenedor de la API no se inició correctamente." -ForegroundColor Yellow
        Write-Host "$ICON_INFO  Revisando los logs de la API:" -ForegroundColor Yellow
        Write-Host "╭───────────────────────────────────────────────────────────╮" -ForegroundColor Cyan
        docker-compose logs api
        Write-Host "╰───────────────────────────────────────────────────────────╯" -ForegroundColor Cyan
        
        if ($Retry -lt $MAX_RETRIES) {
            Write-Host ""
            Write-Host "$ICON_RETRY Reintentando iniciar los contenedores..." -ForegroundColor Yellow
            docker-compose down | Out-Null
            Start-Sleep -Seconds 2
            return Start-Containers -Retry ($Retry + 1)
        } else {
            Write-Host ""
            Write-Host "$ICON_WARNING No se pudo iniciar la API después de $($MAX_RETRIES+1) intentos." -ForegroundColor Red
            return $false
        }
    }
    
    if (-Not $frontendRunning) {
        Write-Host ""
        Write-Host "$ICON_WARNING El contenedor del frontend no se inició correctamente." -ForegroundColor Yellow
        Write-Host "$ICON_INFO  Revisando los logs del frontend:" -ForegroundColor Yellow
        Write-Host "╭───────────────────────────────────────────────────────────╮" -ForegroundColor Cyan
        docker-compose logs frontend
        Write-Host "╰───────────────────────────────────────────────────────────╯" -ForegroundColor Cyan
        
        if ($Retry -lt $MAX_RETRIES) {
            Write-Host ""
            Write-Host "$ICON_RETRY Reintentando iniciar los contenedores..." -ForegroundColor Yellow
            docker-compose down | Out-Null
            Start-Sleep -Seconds 2
            return Start-Containers -Retry ($Retry + 1)
        } else {
            Write-Host ""
            Write-Host "$ICON_WARNING No se pudo iniciar el frontend después de $($MAX_RETRIES+1) intentos." -ForegroundColor Red
            Write-Host "$ICON_INFO  El servicio API y MySQL están disponibles, pero el frontend podría no funcionar correctamente." -ForegroundColor Yellow
            return $false
        }
    }
    
    return $true
}

# Iniciar los contenedores
$success = Start-Containers

# Resumen final con URLs y estado
Write-Host ""
Write-Host "╔════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║                    RESUMEN DE SERVICIOS                    ║" -ForegroundColor Cyan
Write-Host "╠════════════════════════════════════════════════════════════╣" -ForegroundColor Cyan

if ($success) {
    Write-Host "║ $ICON_CHECK ¡Todos los servicios iniciados correctamente!           ║" -ForegroundColor Green
} else {
    Write-Host "║ $ICON_WARNING Servicios iniciados parcialmente                  ║" -ForegroundColor Yellow
}

Write-Host "╠════════════════════════════════════════════════════════════╣" -ForegroundColor Cyan
Write-Host "║ $ICON_WEB Frontend:        http://localhost:3001                  ║" -ForegroundColor Green
Write-Host "║ $ICON_API API:             http://localhost:3000                  ║" -ForegroundColor Green
Write-Host "║ $ICON_DOCS Documentación:  http://localhost:3000/api-docs          ║" -ForegroundColor Green
Write-Host "║ $ICON_DATABASE MySQL:          localhost:3306                           ║" -ForegroundColor Green
Write-Host "╚════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan

Write-Host ""
Write-Host "¡Listo para comenzar a trabajar con AidGuide!" -ForegroundColor Green
Write-Host "$ICON_INFO  Para detener los servicios, ejecute: docker-compose down en el directorio del backend" -ForegroundColor Yellow
Write-Host ""

# Volvemos al directorio original
Set-Location -Path $SCRIPT_DIR 