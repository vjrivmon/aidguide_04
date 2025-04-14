-- Archivo para insertar datos de ejemplo en la base de datos AidGuide
-- Este archivo debe ejecutarse después de crear el esquema
-- === DATOS DE EJEMPLO ===

-- Utilizamos la base de datos AidGuide
USE AidGuide;

-- === Insertar Roles ===
INSERT INTO Roles (nombre) VALUES
('ciego'), ('familiar'), ('administrador'), ('super_usuario');

-- === Insertar Permisos ===
INSERT INTO Permisos (nombre, descripcion) VALUES
('ver_alertas', 'Permite ver las alertas del sistema'),
('controlar_robot', 'Permite enviar comandos al robot'),
('ver_rutas', 'Permite consultar rutas guardadas'),
('administrar_usuarios', 'Permite gestionar cuentas de usuario');

-- === Asignar Permisos a Roles (solo ejemplo simple) ===
INSERT INTO RolPermiso (id_rol, id_permiso) VALUES
(1, 1), (1, 3),
(2, 1),
(3, 1), (3, 2), (3, 3), (3, 4),
(4, 1), (4, 2), (4, 3), (4, 4);

-- === Insertar Usuarios ===
INSERT INTO Usuarios (nombre, apellidos, correo, telefono, huella, contrasena, notificaciones, idioma, volumen, activo, foto_perfil, id_rol)
VALUES
('Juan', 'Pérez', 'juan@example.com', '123456789', 'huella1', 'pass123', true, 'es', 80, true, '', 1),
('Lucía', 'García', 'lucia@example.com', '987654321', 'huella2', 'pass456', true, 'es', 60, true, '', 2),
('Carlos', 'López', 'carlos@example.com', '456123789', 'huella3', 'admin123', false, 'en', 70, true, '', 3);

-- === Insertar Lugares ===
INSERT INTO Lugares (id_usuario, nombre, direccion, favorito, latitud, longitud) VALUES
(1, 'Casa', 'Calle Falsa 123', true, 39.4667, -0.3750),
(1, 'Universidad', 'Av. Universidad 45', false, 39.4750, -0.3600),
(2, 'Oficina', 'Calle Trabajo 88', true, 39.4700, -0.3800);

-- === Insertar Rutas ===
INSERT INTO Rutas (fecha, id_origen, id_destino, duracion, mapa, ultimo_uso, descripcion, completada, id_usuario) VALUES
(NOW(), 1, 2, 15, 'mapa1.png', NOW(), 'Ruta diaria a la universidad', true, 1),
(NOW(), 3, 1, 20, 'mapa2.png', NOW(), 'Vuelta a casa', false, 2);

-- === Insertar Robots ===
INSERT INTO Robots (nombre, modelo, version_firmware, bateria, conexion, estado, averias, ubicacion, velocidad_navegacion, id_usuario) VALUES
('AidBot_01', 'Modelo-A', 'v1.0', 90, 'WiFi', 'Activo', '', 'Universidad', 'media', 1),
('AidBot_02', 'Modelo-B', 'v1.1', 75, 'LTE', 'En espera', 'sensor fallido', 'Casa', 'lenta', 2);

-- === Insertar Categorías ===
INSERT INTO Categorias (tipo, descripcion, ubicacion, imagen) VALUES
('persona', 'Persona detectada en la vía', 'Calle Mayor', 'persona.jpg'),
('señal de tráfico', 'Señal de STOP', 'Av. Universidad', 'stop.jpg'),
('paso de peatones', 'Paso peatonal frente a facultad', 'Campus', 'peaton.jpg');

-- === Insertar Detecciones ===
INSERT INTO Detecciones (id_robot, id_ruta, id_categoria, fecha_hora) VALUES
(1, 1, 1, NOW()),
(1, 1, 2, NOW()),
(2, 2, 3, NOW());

-- === Insertar Historial de Ubicaciones ===
INSERT INTO HistorialUbicaciones (id_robot, fecha_hora, latitud, longitud, descripcion, evento) VALUES
(1, NOW(), 39.4667, -0.3750, 'Inicio de ruta', 'inicio'),
(1, NOW(), 39.4675, -0.3740, 'Cruce detectado', 'cruce'),
(2, NOW(), 39.4700, -0.3800, 'Inicio de patrulla', 'inicio');

-- === Insertar Alertas ===
INSERT INTO Alertas (id_robot, id_usuario, id_deteccion, tipo, mensaje, prioridad, leida, fecha_hora) VALUES
(1, 1, 1, 'obstaculo', 'Obstáculo detectado en ruta 1', 'alta', false, NOW()),
(2, 2, 3, 'bateria', 'Batería baja detectada en AidBot_02', 'media', false, NOW());