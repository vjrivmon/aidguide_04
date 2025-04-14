# Script de automatización para iniciar el paquete de navegación aidguide_04_nav
# Autor: DevOps Team
# Fecha: 2024
# Descripción: Este script automatiza la compilación y ejecución del paquete de navegación

Write-Host "🚀 Iniciando el proceso de configuración de navegación..." -ForegroundColor Cyan

# 1. Nos situamos en la carpeta del workspace
Write-Host "📂 Navegando al workspace..." -ForegroundColor Yellow
Set-Location -Path "~/aidguide_04/aidguide_04_ws"

# 2. Compilamos el paquete
Write-Host "🔨 Compilando el paquete..." -ForegroundColor Yellow
colcon build --packages-select aidguide_04_nav

# 3. Verificamos y establecemos permisos de los archivos Python
Write-Host "🔑 Verificando permisos de archivos..." -ForegroundColor Yellow
$navPath = "src/aidguide_04_nav/aidguide_04_nav"
$launchPath = "src/aidguide_04_nav/launch"

# En Windows no es tan crítico el tema de permisos, pero igualmente verificamos que existan los archivos
if (Test-Path "$navPath/aidguide_04_nav.py") {
    Write-Host "✅ aidguide_04_nav.py encontrado" -ForegroundColor Green
} else {
    Write-Host "❌ No se encuentra aidguide_04_nav.py" -ForegroundColor Red
    exit 1
}

if (Test-Path "$launchPath/aidguide_04_nav_launch.launch.py") {
    Write-Host "✅ Launch file encontrado" -ForegroundColor Green
} else {
    Write-Host "❌ No se encuentra el archivo launch" -ForegroundColor Red
    exit 1
}

# 4. Iniciamos la simulación en una nueva ventana
Write-Host "🌐 Iniciando simulación Gazebo..." -ForegroundColor Cyan
Start-Process pwsh -ArgumentList "-NoExit", "-Command", "ros2 launch turtlebot3_gazebo empty_world.launch.py"

# 5. Esperamos unos segundos para que Gazebo se inicie
Write-Host "⏳ Esperando a que Gazebo se inicie..." -ForegroundColor Yellow
Start-Sleep -Seconds 10

# 6. Iniciamos la navegación
Write-Host "🗺️ Iniciando sistema de navegación..." -ForegroundColor Cyan
& $env:SYSTEMROOT\System32\WindowsPowerShell\v1.0\powershell.exe -Command {
    Set-Location -Path "~/aidguide_04/aidguide_04_ws"
    . install/setup.ps1
    ros2 launch aidguide_04_nav aidguide_04_nav_launch.launch.py
} 