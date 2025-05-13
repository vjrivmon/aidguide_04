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

# Función para comprobar y crear el archivo .env si no existe
function Check-EnvFile {
    $envPath = Join-Path $BACKEND_PATH ".env"
    if (-Not (Test-Path -Path $envPath)) {
        Write-Host "$ICON_INFO  Archivo .env no encontrado, creando uno por defecto..." -ForegroundColor Yellow
        $envContent = @"
# Configuración de la base de datos
DB_HOST=mysql
DB_USER=aiduser
DB_PASSWORD=password123
DB_NAME=AidGuide
DB_PORT=3306

# Configuración de autenticación
JWT_SECRET=aidguide_secure_secret_key_2024
JWT_REFRESH_SECRET=aidguide_secure_refresh_key_2024

# Configuración del servidor
PORT=3000
NODE_ENV=development
"@
        $envContent | Out-File -FilePath $envPath -Encoding utf8
        Write-Host "$ICON_CHECK  Archivo .env creado con éxito" -ForegroundColor Green
    } else {
        Write-Host "$ICON_CHECK  Archivo .env encontrado" -ForegroundColor Green
    }
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
    $response = Read-Host "¿Deseas iniciar Docker Desktop ahora? (s/n)"
    if ($response -eq "s") {
        Write-Host "$ICON_LOADING  Iniciando Docker Desktop..." -ForegroundColor Yellow
        Start-Process "C:\Program Files\Docker\Docker\Docker Desktop.exe"
        Write-Host "$ICON_INFO  Esperando a que Docker se inicie (30 segundos)..." -ForegroundColor Yellow
        
        $dockerStarted = $false
        for ($i = 0; $i -lt 10; $i++) {
            Start-Sleep -Seconds 3
            Write-Host "." -NoNewline -ForegroundColor Cyan
            try {
                docker info | Out-Null
                $dockerStarted = $true
                break
            } catch {
                # Sigue esperando
            }
        }
        
        if ($dockerStarted) {
            Write-Host ""
            Write-Host "$ICON_CHECK Docker iniciado correctamente" -ForegroundColor Green
        } else {
            Write-Host ""
            Write-Host "$ICON_ERROR No se pudo iniciar Docker automáticamente. Por favor, inicia Docker Desktop manualmente y vuelve a ejecutar este script." -ForegroundColor Red
            exit 1
        }
    } else {
        Write-Host "$ICON_ERROR Docker es necesario para ejecutar los servicios. Por favor, inicia Docker Desktop e intenta nuevamente." -ForegroundColor Red
        exit 1
    }
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

# Verificar y crear el archivo .env si no existe
Check-EnvFile
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
    $mysqlRunning = docker ps --filter "name=mysql_aidguide" --format "{{.Names}}" | Select-String -Pattern "mysql_aidguide"
    $apiRunning = docker ps --filter "name=aidguide_api" --format "{{.Names}}" | Select-String -Pattern "aidguide_api"
    $frontendRunning = docker ps --filter "name=aidguide_frontend" --format "{{.Names}}" | Select-String -Pattern "aidguide_frontend"
    
    # Verificar MySQL
    if (-Not $mysqlRunning) {
        Write-Host ""
        Write-Host "$ICON_ERROR MySQL no se inició correctamente." -ForegroundColor Red
        Write-Host "$ICON_INFO  Revisando los logs de MySQL:" -ForegroundColor Yellow
        Write-Host "╭───────────────────────────────────────────────────────────╮" -ForegroundColor Cyan
        docker-compose logs mysql
        Write-Host "╰───────────────────────────────────────────────────────────╯" -ForegroundColor Cyan
        
        if ($Retry -lt $MAX_RETRIES) {
            Write-Host ""
            Write-Host "$ICON_RETRY Reintentando iniciar los contenedores..." -ForegroundColor Yellow
            docker-compose down | Out-Null
            Start-Sleep -Seconds 2
            return Start-Containers -Retry ($Retry + 1)
        } else {
            Write-Host ""
            Write-Host "$ICON_ERROR No se pudo iniciar MySQL después de $($MAX_RETRIES+1) intentos." -ForegroundColor Red
            return $false
        }
    }
    
    # Verificar API
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
            Write-Host "$ICON_INFO  ¿Deseas iniciar solo la base de datos para desarrollo local? (s/n)" -ForegroundColor Yellow
            $response = Read-Host
            if ($response -eq "s") {
                Write-Host "$ICON_LOADING Iniciando solo la base de datos..." -ForegroundColor Cyan
                docker-compose up -d mysql
                Show-Progress -Duration 2 -Message "$ICON_LOADING Inicializando base de datos..."
                return $true, $false, $false
            }
            return $false
        }
    }
    
    # Verificar Frontend
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
            Write-Host "$ICON_INFO  La API y MySQL están disponibles, pero el frontend podría no funcionar correctamente." -ForegroundColor Yellow
            return $true, $true, $false
        }
    }
    
    # Probar conectividad a los servicios
    $apiAccessible = $false
    $frontendAccessible = $false
    
    # Probar API
    Write-Host ""
    Write-Host "$ICON_INFO  Comprobando conectividad con la API..." -ForegroundColor Yellow
    try {
        $response = Invoke-WebRequest -Uri "http://localhost:3333/health" -TimeoutSec 5 -ErrorAction SilentlyContinue
        if ($response.StatusCode -eq 200) {
            Write-Host "$ICON_CHECK API accesible en http://localhost:3333" -ForegroundColor Green
            $apiAccessible = $true
        }
    } catch {
        Write-Host "$ICON_WARNING API en ejecución pero podría no ser accesible. Revisa los logs para más detalles." -ForegroundColor Yellow
    }
    
    # Probar Frontend
    Write-Host "$ICON_INFO  Comprobando conectividad con el Frontend..." -ForegroundColor Yellow
    try {
        $response = Invoke-WebRequest -Uri "http://localhost:3334" -TimeoutSec 5 -ErrorAction SilentlyContinue
        if ($response.StatusCode -eq 200) {
            Write-Host "$ICON_CHECK Frontend accesible en http://localhost:3334" -ForegroundColor Green
            $frontendAccessible = $true
        }
    } catch {
        Write-Host "$ICON_WARNING Frontend en ejecución pero podría no ser accesible. Revisa los logs para más detalles." -ForegroundColor Yellow
    }
    
    return $true, $apiAccessible, $frontendAccessible
}

# Iniciar los contenedores
$result = Start-Containers
if ($result -is [array]) {
    $success = $result[0]
    $apiOK = $result[1]
    $frontendOK = $result[2]
} else {
    $success = $result
    $apiOK = $false
    $frontendOK = $false
}

# Resumen final con URLs y estado
Write-Host ""
Write-Host "╔════════════════════════════════════════════════════════════╗" -ForegroundColor Cyan
Write-Host "║                    RESUMEN DE SERVICIOS                    ║" -ForegroundColor Cyan
Write-Host "╠════════════════════════════════════════════════════════════╣" -ForegroundColor Cyan

if ($success -and $apiOK -and $frontendOK) {
    Write-Host "║ $ICON_CHECK ¡Todos los servicios iniciados correctamente!           ║" -ForegroundColor Green
} elseif ($success) {
    Write-Host "║ $ICON_WARNING Servicios iniciados parcialmente                  ║" -ForegroundColor Yellow
} else {
    Write-Host "║ $ICON_ERROR No se pudieron iniciar los servicios                  ║" -ForegroundColor Red
}

Write-Host "╠════════════════════════════════════════════════════════════╣" -ForegroundColor Cyan
Write-Host "║ $ICON_WEB Frontend:        http://localhost:3334                  ║" -ForegroundColor $(if ($frontendOK) { "Green" } else { "DarkGray" })
Write-Host "║ $ICON_API API:             http://localhost:3333                  ║" -ForegroundColor $(if ($apiOK) { "Green" } else { "DarkGray" })
Write-Host "║ $ICON_DOCS Documentación:  http://localhost:3333/api-docs          ║" -ForegroundColor $(if ($apiOK) { "Green" } else { "DarkGray" })
Write-Host "║ $ICON_DATABASE MySQL:          localhost:3306                           ║" -ForegroundColor Green
Write-Host "╚════════════════════════════════════════════════════════════╝" -ForegroundColor Cyan

Write-Host ""
Write-Host "¡Listo para comenzar a trabajar con AidGuide!" -ForegroundColor Green
Write-Host "$ICON_INFO  Comandos útiles:" -ForegroundColor Yellow
Write-Host " - Para ver los logs: $ICON_LOADING docker-compose logs" -ForegroundColor Cyan
Write-Host " - Para detener los servicios: $ICON_CLEANING docker-compose down" -ForegroundColor Cyan
Write-Host " - Para reiniciar un servicio: $ICON_RETRY docker-compose restart <servicio>" -ForegroundColor Cyan
Write-Host ""

# Volvemos al directorio original
Set-Location -Path $SCRIPT_DIR 