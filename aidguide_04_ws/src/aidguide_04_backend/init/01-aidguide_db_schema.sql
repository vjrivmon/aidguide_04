-- Archivo para la creación del esquema de la base de datos AidGuide
-- Este archivo debe ejecutarse antes de insertar los datos de ejemplo
-- === ESQUEMA DE LA BASE DE DATOS ===

-- Utilizamos la base de datos AidGuide
USE AidGuide;

-- Tabla de Roles
CREATE TABLE IF NOT EXISTS Roles (
  id_rol INT PRIMARY KEY AUTO_INCREMENT,
  nombre VARCHAR(15)
);

-- Tabla de Permisos
CREATE TABLE IF NOT EXISTS Permisos (
  id_permiso INT PRIMARY KEY AUTO_INCREMENT,
  nombre VARCHAR(50),
  descripcion VARCHAR(255)
);

-- Tabla intermedia RolPermiso
CREATE TABLE IF NOT EXISTS RolPermiso (
  id_rol INT,
  id_permiso INT,
  PRIMARY KEY (id_rol, id_permiso),
  FOREIGN KEY (id_rol) REFERENCES Roles(id_rol),
  FOREIGN KEY (id_permiso) REFERENCES Permisos(id_permiso)
);

-- Tabla de Usuarios
CREATE TABLE IF NOT EXISTS Usuarios (
  id_usuario INT PRIMARY KEY AUTO_INCREMENT,
  nombre VARCHAR(50),
  apellidos VARCHAR(100),
  correo VARCHAR(50),
  telefono VARCHAR(20),
  huella VARCHAR(50),
  contrasena VARCHAR(100),
  notificaciones BOOLEAN,
  idioma VARCHAR(25),
  volumen INT,
  activo BOOLEAN,
  foto_perfil VARCHAR(200),
  id_rol INT,
  FOREIGN KEY (id_rol) REFERENCES Roles(id_rol)
);

-- Tabla de Lugares
CREATE TABLE IF NOT EXISTS Lugares (
  id_lugar INT PRIMARY KEY AUTO_INCREMENT,
  id_usuario INT,
  nombre VARCHAR(50),
  direccion VARCHAR(255),
  favorito BOOLEAN,
  latitud DECIMAL(9,6),
  longitud DECIMAL(9,6),
  FOREIGN KEY (id_usuario) REFERENCES Usuarios(id_usuario)
);

-- Tabla de Rutas
CREATE TABLE IF NOT EXISTS Rutas (
  id_ruta INT PRIMARY KEY AUTO_INCREMENT,
  fecha DATETIME,
  id_origen INT,
  id_destino INT,
  duracion INT,
  mapa VARCHAR(100),
  ultimo_uso DATETIME,
  descripcion VARCHAR(255),
  completada BOOLEAN,
  id_usuario INT,
  FOREIGN KEY (id_origen) REFERENCES Lugares(id_lugar),
  FOREIGN KEY (id_destino) REFERENCES Lugares(id_lugar),
  FOREIGN KEY (id_usuario) REFERENCES Usuarios(id_usuario)
);

-- Tabla de Robots
CREATE TABLE IF NOT EXISTS Robots (
  id_robot INT PRIMARY KEY AUTO_INCREMENT,
  nombre VARCHAR(50),
  modelo VARCHAR(50),
  version_firmware VARCHAR(20),
  bateria INT,
  conexion VARCHAR(25),
  estado VARCHAR(25),
  averias VARCHAR(25),
  ubicacion VARCHAR(100),
  velocidad_navegacion VARCHAR(25),
  id_usuario INT,
  FOREIGN KEY (id_usuario) REFERENCES Usuarios(id_usuario)
);

-- Tabla de Categorías
CREATE TABLE IF NOT EXISTS Categorias (
  id_categoria INT PRIMARY KEY AUTO_INCREMENT,
  tipo VARCHAR(50),
  descripcion VARCHAR(255),
  ubicacion VARCHAR(100),
  imagen VARCHAR(200)
);

-- Tabla de Detecciones
CREATE TABLE IF NOT EXISTS Detecciones (
  id_deteccion INT PRIMARY KEY AUTO_INCREMENT,
  id_robot INT,
  id_ruta INT,
  id_categoria INT,
  fecha_hora DATETIME,
  FOREIGN KEY (id_robot) REFERENCES Robots(id_robot),
  FOREIGN KEY (id_ruta) REFERENCES Rutas(id_ruta),
  FOREIGN KEY (id_categoria) REFERENCES Categorias(id_categoria)
);

-- Tabla de Historial de Ubicaciones
CREATE TABLE IF NOT EXISTS HistorialUbicaciones (
  id_historial INT PRIMARY KEY AUTO_INCREMENT,
  id_robot INT,
  fecha_hora DATETIME,
  latitud DECIMAL(9,6),
  longitud DECIMAL(9,6),
  descripcion VARCHAR(100),
  evento VARCHAR(50),
  FOREIGN KEY (id_robot) REFERENCES Robots(id_robot)
);

-- Tabla de Alertas
CREATE TABLE IF NOT EXISTS Alertas (
  id_alerta INT PRIMARY KEY AUTO_INCREMENT,
  id_robot INT,
  id_usuario INT,
  id_deteccion INT,
  tipo VARCHAR(50),
  mensaje VARCHAR(255),
  prioridad ENUM('alta', 'media', 'baja'),
  leida BOOLEAN,
  fecha_hora DATETIME,
  FOREIGN KEY (id_robot) REFERENCES Robots(id_robot),
  FOREIGN KEY (id_usuario) REFERENCES Usuarios(id_usuario),
  FOREIGN KEY (id_deteccion) REFERENCES Detecciones(id_deteccion)
);
