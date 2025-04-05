# AidGuide 04 - Backend

## Descripción General

Este repositorio contiene el backend completo para el sistema AidGuide, una plataforma de asistencia para guiado robótico. El backend proporciona una API RESTful que permite gestionar usuarios, robots, rutas, lugares y más, sirviendo como capa intermedia entre el frontend de la aplicación y el sistema ROS2 que controla los robots.

## Estructura del Proyecto

```
aidguide_04_backend/
├── api/                     # Código fuente de la API REST
│   ├── routes/              # Controladores de rutas por entidad
│   │   ├── usuarios.js      # Endpoints para gestión de usuarios
│   │   ├── roles.js         # Endpoints para gestión de roles
│   │   ├── lugares.js       # Endpoints para gestión de lugares
│   │   ├── robots.js        # Endpoints para gestión de robots
│   │   └── rutas.js         # Endpoints para gestión de rutas
│   ├── swagger.json         # Documentación de la API con Swagger
│   ├── db.js                # Configuración de conexión a la base de datos
│   ├── app.js               # Configuración principal de Express
│   ├── Dockerfile           # Configuración para construcción de imagen Docker
│   └── package.json         # Dependencias y configuración de Node.js
├── init/                    # Scripts de inicialización para la base de datos
│   ├── 01-aidguide_db_schema.sql    # Definición del esquema de la BD
│   └── 02-aidguide_fake_data.sql    # Datos de ejemplo para desarrollo
└── docker-compose.yml       # Configuración de despliegue con Docker Compose (incluye frontend)
```

## Características Principales

### 1. API RESTful Completa

El backend proporciona una API RESTful completa con los siguientes endpoints:

- **/api/usuarios**: Gestión de usuarios (CRUD)
- **/api/roles**: Gestión de roles de usuario (CRUD)
- **/api/lugares**: Gestión de lugares y ubicaciones (CRUD)
- **/api/robots**: Gestión de robots y su estado (CRUD)
- **/api/rutas**: Gestión de rutas de navegación (CRUD)
- **/api/status**: Estado del servicio

### 2. Modelo de Datos Completo

Se ha implementado un esquema de base de datos MySQL completo con las siguientes entidades:

- **Usuarios**: Datos de los usuarios del sistema
- **Roles**: Roles y permisos para usuarios
- **Lugares**: Ubicaciones y puntos de interés
- **Robots**: Información y estado de robots disponibles
- **Rutas**: Trayectos entre ubicaciones
- **Categorías**: Tipos de objetos detectables
- **Detecciones**: Registro de objetos detectados
- **Historial de Ubicaciones**: Seguimiento de la posición del robot
- **Alertas**: Sistema de notificaciones y alertas

### 3. Documentación con Swagger

Toda la API está documentada utilizando Swagger, accesible en **/api-docs**, lo que permite:

- Explorar todos los endpoints disponibles
- Probar las operaciones directamente desde el navegador
- Obtener información detallada sobre parámetros y respuestas
- Generar clientes automáticamente para diferentes lenguajes

### 4. Contenerización Completa del Sistema

El sistema completo está contenerizado utilizando Docker y Docker Compose:

- **Servicio MySQL**: Base de datos MySQL 8.0 con persistencia de datos
- **Servicio API**: Servidor Node.js con Express
- **Servicio Frontend**: Aplicación web Next.js con interfaz de usuario
- Redes y volúmenes configurados para desarrollo y producción
- Scripts de inicialización automática
- Healthchecks para garantizar la disponibilidad de los servicios

## Requisitos

- Docker y Docker Compose
- Node.js 14+ (para desarrollo local)
- npm o yarn (para desarrollo local)

## Instalación y Ejecución

### Usando Docker Compose (Recomendado)

1. Clona el repositorio
2. Navega a la carpeta del backend
3. Ejecuta Docker Compose:

```bash
cd aidguide_04_ws/src/aidguide_04_backend
docker-compose up -d
```

La API estará disponible en http://localhost:3000 y la documentación en http://localhost:3000/api-docs
El frontend estará disponible en http://localhost:3001

### Desarrollo Local

1. Inicia la base de datos con Docker:

```bash
docker-compose up -d mysql
```

2. Instala las dependencias y ejecuta la API:

```bash
cd api
npm install
npm run dev
```

## Protocolos de Comunicación

### API REST

La comunicación con el frontend se realiza a través de la API REST, utilizando JSON como formato de intercambio de datos. Todos los endpoints siguen las convenciones RESTful:

- **GET**: Obtener recursos
- **POST**: Crear recursos
- **PUT**: Actualizar recursos
- **DELETE**: Eliminar recursos

### Integración con ROS2

El backend está diseñado para integrarse con el sistema ROS2 Galactic mediante el uso de interfaces de comunicación específicas:

1. **Comunicación bidireccional**: 
   - El backend recibe actualizaciones de estado en tiempo real de los robots
   - Envía comandos de navegación y control al sistema ROS2

2. **Mensajes y servicios ROS2**:
   - Utiliza mensajes personalizados para datos de estado del robot
   - Implementa servicios para solicitar operaciones específicas
   - Soporta topics para actualizaciones asíncronas

3. **Almacenamiento y persistencia**:
   - Almacena el historial de ubicaciones para análisis
   - Registra todas las interacciones en la base de datos
   - Mantiene un log detallado de eventos y alertas

4. **Arquitectura modular**:
   - Permite agregar nuevos tipos de robots sin modificar la estructura base
   - Soporta distintos protocolos de comunicación según el tipo de robot
   - Facilita la implementación de nuevas funcionalidades mediante plugins

## Desarrollo y Contribución

### Agregar Nuevos Endpoints

1. Crea un nuevo archivo en la carpeta `api/routes/`
2. Implementa los endpoints siguiendo el patrón de los existentes
3. Registra las rutas en `app.js`
4. Actualiza la documentación Swagger en `swagger.json`

### Modificar el Esquema de la Base de Datos

1. Actualiza el archivo `init/01-aidguide_db_schema.sql`
2. Actualiza los datos de ejemplo en `init/02-aidguide_fake_data.sql`
3. Reconstruye los contenedores:

```bash
docker-compose down -v
docker-compose up -d
```

### Actualizar el README

Es importante mantener este documento actualizado cuando se implementen cambios significativos:

1. **Nuevos endpoints**: Agrega cualquier endpoint nuevo a la sección "API RESTful Completa"
2. **Cambios en el modelo de datos**: Actualiza la sección "Modelo de Datos Completo"
3. **Nuevas dependencias**: Refleja las nuevas dependencias en la sección "Requisitos"
4. **Cambios en la estructura**: Actualiza el diagrama de la estructura del proyecto
5. **Nuevas funcionalidades**: Marca las nuevas características en "Estado Actual y Próximos Pasos"

## Seguridad

- La API utiliza MySQL con autenticación nativa
- Todas las consultas SQL están parametrizadas para prevenir inyección SQL
- Se implementa CORS para controlar el acceso desde dominios externos
- Las contraseñas y datos sensibles se deben gestionar de forma segura

## Estado Actual y Próximos Pasos

- [x] Implementación del CRUD básico para todas las entidades
- [x] Documentación con Swagger
- [x] Contenerización completa con Docker
- [x] Integración con frontend mediante docker-compose
- [x] Configuración de redes Docker para comunicación entre servicios
- [ ] Implementación de autenticación JWT
- [ ] Integración directa con nodos ROS2
- [ ] Implementación de WebSockets para comunicación en tiempo real
- [ ] Pruebas automatizadas para todos los endpoints
- [ ] Implementación de sistema de logs centralizado
- [ ] Monitorización de servicios con Prometheus y Grafana

## Contacto

Para más información o consultas, contactar con el equipo de desarrollo de AidGuide.

---

*Este documento fue generado para el proyecto AidGuide 04, parte del proyecto de robótica de GTI de la Universidad Politécnica de Valencia.* 