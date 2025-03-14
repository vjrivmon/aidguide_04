# AidGuide 04

Sistema de guiado asistido para robots móviles basado en ROS2 Galactic.

## Estructura del Proyecto
El proyecto está organizado en los siguientes componentes principales:
- `src/custom_interface`: Interfaces personalizadas para la comunicación entre nodos
- `src/aidguide_04`: Nodo principal del sistema de guiado

## Requisitos
- ROS2 Galactic
- Python 3.8+
- Dependencias específicas listadas en package.xml

## Instalación
1. Clonar el repositorio:
```bash
git clone https://github.com/vjrivmon/aidguide_04.git
cd aidguide_04
```

2. Instalar dependencias:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

3. Compilar el workspace:
```bash
colcon build
```

## Uso
Para ejecutar el sistema:
```bash
ros2 launch aidguide_04 main.launch.py
```

## Equipo de Desarrollo
- Vicente Rivas Monferrer (Responsable) - vjrivmon@epsg.upv.es
- Irene Medina García
- Mimi Vladeva
- Hugo Belda Revert
- Marc Vilagrosa Caturla

## Contribución
Por favor, lee [CONTRIBUTING.md](docs/CONTRIBUTING.md) para detalles sobre nuestro código de conducta y el proceso para enviar pull requests.

## Licencia
Este proyecto está bajo la Licencia MIT - ver el archivo [LICENSE](LICENSE) para más detalles. 