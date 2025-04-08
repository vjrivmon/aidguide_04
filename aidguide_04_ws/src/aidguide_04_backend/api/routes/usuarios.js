const express = require('express');
const router = express.Router();
const db = require('../db');
const bcrypt = require('bcrypt');
const jwt = require('jsonwebtoken');
const { v4: uuidv4 } = require('uuid');

// Middleware para verificar token JWT
const verificarToken = (req, res, next) => {
  const token = req.headers['authorization']?.split(' ')[1];
  
  if (!token) {
    return res.status(403).json({ error: 'No se proporcionó token de autenticación' });
  }
  
  try {
    const decoded = jwt.verify(token, process.env.JWT_SECRET || 'clave_secreta_temporal');
    req.usuario = decoded;
    next();
  } catch (error) {
    return res.status(401).json({ error: 'Token inválido o expirado' });
  }
};

// GET /api/usuarios - Obtener todos los usuarios
router.get('/', (req, res) => {
  const query = 'SELECT u.*, r.nombre as rol FROM Usuarios u LEFT JOIN Roles r ON u.id_rol = r.id_rol';
  db.query(query, (err, results) => {
    if (err) {
      console.error('Error al obtener usuarios:', err);
      return res.status(500).json({ error: 'Error al obtener usuarios' });
    }
    res.json(results);
  });
});

// GET /api/usuarios/:id - Obtener un usuario por ID
router.get('/:id', (req, res) => {
  const { id } = req.params;
  const query = 'SELECT u.*, r.nombre as rol FROM Usuarios u LEFT JOIN Roles r ON u.id_rol = r.id_rol WHERE u.id_usuario = ?';
  db.query(query, [id], (err, results) => {
    if (err) {
      console.error('Error al obtener usuario:', err);
      return res.status(500).json({ error: 'Error al obtener usuario' });
    }
    if (results.length === 0) {
      return res.status(404).json({ error: 'Usuario no encontrado' });
    }
    res.json(results[0]);
  });
});

// Obtener perfil del usuario autenticado
router.get('/perfil/actual', verificarToken, (req, res) => {
  const userId = req.usuario.id;
  const query = 'SELECT u.*, r.nombre as rol, p.nivel, p.puntos FROM Usuarios u LEFT JOIN Roles r ON u.id_rol = r.id_rol LEFT JOIN ProgresoUsuario p ON u.id_usuario = p.id_usuario WHERE u.id_usuario = ?';
  
  db.query(query, [userId], (err, results) => {
    if (err) {
      console.error('Error al obtener perfil:', err);
      return res.status(500).json({ error: 'Error al obtener perfil de usuario' });
    }
    if (results.length === 0) {
      return res.status(404).json({ error: 'Usuario no encontrado' });
    }
    
    // Eliminar la contraseña del resultado
    const usuario = results[0];
    delete usuario.contrasena;
    delete usuario.password_hash;
    
    res.json(usuario);
  });
});

// POST /api/usuarios - Crear un nuevo usuario
router.post('/', (req, res) => {
  const { 
    nombre, apellidos, correo, telefono, huella, contrasena, 
    notificaciones, idioma, volumen, activo, foto_perfil, id_rol 
  } = req.body;
  
  if (!nombre || !correo || !contrasena || !id_rol) {
    return res.status(400).json({ error: 'Nombre, correo, contraseña y rol son obligatorios' });
  }
  
  const query = `
    INSERT INTO Usuarios 
    (nombre, apellidos, correo, telefono, huella, contrasena, notificaciones, idioma, volumen, activo, foto_perfil, id_rol) 
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`;
  
  const values = [
    nombre, apellidos || null, correo, telefono || null, huella || null, 
    contrasena, notificaciones || false, idioma || 'es', volumen || 50, 
    activo !== undefined ? activo : true, foto_perfil || null, id_rol
  ];
  
  db.query(query, values, (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    
    res.status(201).json({
      id_usuario: result.insertId,
      nombre,
      apellidos,
      correo,
      telefono,
      huella,
      notificaciones,
      idioma,
      volumen,
      activo,
      foto_perfil,
      id_rol
    });
  });
});

// PUT /api/usuarios/:id - Actualizar un usuario
router.put('/:id', (req, res) => {
  const { 
    nombre, apellidos, correo, telefono, huella, contrasena, 
    notificaciones, idioma, volumen, activo, foto_perfil, id_rol 
  } = req.body;
  
  // Primero verificamos si el usuario existe
  db.query('SELECT * FROM Usuarios WHERE id_usuario = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Usuario no encontrado' });
    
    const usuario = results[0];
    
    // Preparamos los valores actualizados
    const updatedUser = {
      nombre: nombre || usuario.nombre,
      apellidos: apellidos || usuario.apellidos,
      correo: correo || usuario.correo,
      telefono: telefono || usuario.telefono,
      huella: huella || usuario.huella,
      contrasena: contrasena || usuario.contrasena,
      notificaciones: notificaciones !== undefined ? notificaciones : usuario.notificaciones,
      idioma: idioma || usuario.idioma,
      volumen: volumen !== undefined ? volumen : usuario.volumen,
      activo: activo !== undefined ? activo : usuario.activo,
      foto_perfil: foto_perfil || usuario.foto_perfil,
      id_rol: id_rol || usuario.id_rol
    };
    
    const query = `
      UPDATE Usuarios 
      SET nombre = ?, apellidos = ?, correo = ?, telefono = ?, 
          huella = ?, contrasena = ?, notificaciones = ?, idioma = ?, 
          volumen = ?, activo = ?, foto_perfil = ?, id_rol = ?
      WHERE id_usuario = ?`;
    
    const values = [
      updatedUser.nombre, updatedUser.apellidos, updatedUser.correo,
      updatedUser.telefono, updatedUser.huella, updatedUser.contrasena,
      updatedUser.notificaciones, updatedUser.idioma, updatedUser.volumen,
      updatedUser.activo, updatedUser.foto_perfil, updatedUser.id_rol,
      req.params.id
    ];
    
    db.query(query, values, (err, result) => {
      if (err) return res.status(500).json({ error: err.message });
      
      res.json({
        id_usuario: parseInt(req.params.id),
        ...updatedUser
      });
    });
  });
});

// DELETE /api/usuarios/:id - Eliminar un usuario
router.delete('/:id', (req, res) => {
  db.query('DELETE FROM Usuarios WHERE id_usuario = ?', [req.params.id], (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    if (result.affectedRows === 0) return res.status(404).json({ error: 'Usuario no encontrado' });
    res.json({ message: 'Usuario eliminado con éxito' });
  });
});

// Registro de usuario
router.post('/registro', async (req, res) => {
  const { nombre, correo, contrasena } = req.body;
  
  if (!nombre || !correo || !contrasena) {
    return res.status(400).json({ error: 'Todos los campos son obligatorios' });
  }
  
  try {
    // Verificar si el correo ya existe
    const checkEmailQuery = 'SELECT * FROM Autenticacion WHERE email = ?';
    db.query(checkEmailQuery, [correo], async (err, results) => {
      if (err) {
        console.error('Error al verificar correo:', err);
        return res.status(500).json({ error: 'Error en el registro' });
      }
      
      if (results.length > 0) {
        return res.status(400).json({ error: 'El correo ya está registrado' });
      }
      
      // Generar hash de contraseña
      const saltRounds = 10;
      const passwordHash = await bcrypt.hash(contrasena, saltRounds);
      
      // Generar código de verificación y vinculación
      const codigoVerificacion = Math.floor(100000 + Math.random() * 900000).toString();
      const codigoVinculacion = Math.floor(1000000000 + Math.random() * 9000000000).toString().substring(0, 10);
      
      // Iniciar transacción
      db.beginTransaction(async err => {
        if (err) {
          console.error('Error al iniciar transacción:', err);
          return res.status(500).json({ error: 'Error en el registro' });
        }
        
        // Insertar en tabla Autenticacion
        const authQuery = 'INSERT INTO Autenticacion (email, password_hash, codigo_verificacion) VALUES (?, ?, ?)';
        db.query(authQuery, [correo, passwordHash, codigoVerificacion], (err, authResult) => {
          if (err) {
            return db.rollback(() => {
              console.error('Error al crear autenticación:', err);
              res.status(500).json({ error: 'Error en el registro' });
            });
          }
          
          const idAutenticacion = authResult.insertId;
          
          // Insertar en tabla Usuarios
          const userQuery = 'INSERT INTO Usuarios (nombre, correo, id_rol, id_autenticacion, codigo_vinculacion, activo) VALUES (?, ?, 2, ?, ?, true)';
          db.query(userQuery, [nombre, correo, idAutenticacion, codigoVinculacion], (err, userResult) => {
            if (err) {
              return db.rollback(() => {
                console.error('Error al crear usuario:', err);
                res.status(500).json({ error: 'Error en el registro' });
              });
            }
            
            const idUsuario = userResult.insertId;
            
            // Insertar registro en ProgresoUsuario
            const progresoQuery = 'INSERT INTO ProgresoUsuario (id_usuario, puntos, nivel) VALUES (?, 0, 1)';
            db.query(progresoQuery, [idUsuario], (err) => {
              if (err) {
                return db.rollback(() => {
                  console.error('Error al crear progreso de usuario:', err);
                  res.status(500).json({ error: 'Error en el registro' });
                });
              }
              
              // Confirmar transacción
              db.commit(err => {
                if (err) {
                  return db.rollback(() => {
                    console.error('Error al confirmar transacción:', err);
                    res.status(500).json({ error: 'Error en el registro' });
                  });
                }
                
                // Devolver resultado exitoso
                res.status(201).json({
                  mensaje: 'Usuario registrado correctamente',
                  id_usuario: idUsuario,
                  codigo_vinculacion: codigoVinculacion
                });
                
                // Aquí se podría enviar email con el código de verificación
              });
            });
          });
        });
      });
    });
  } catch (error) {
    console.error('Error en registro:', error);
    res.status(500).json({ error: 'Error en el registro' });
  }
});

// Iniciar sesión
router.post('/login', async (req, res) => {
  const { email, password } = req.body;
  
  if (!email || !password) {
    return res.status(400).json({ error: 'Correo y contraseña son obligatorios' });
  }
  
  try {
    // Consultar usuario por correo
    const query = `
      SELECT a.*, u.id_usuario, u.nombre, u.id_rol, r.nombre as rol_nombre
      FROM Autenticacion a
      JOIN Usuarios u ON a.id_autenticacion = u.id_autenticacion
      LEFT JOIN Roles r ON u.id_rol = r.id_rol
      WHERE a.email = ?
    `;
    
    db.query(query, [email], async (err, results) => {
      if (err) {
        console.error('Error al consultar usuario:', err);
        return res.status(500).json({ error: 'Error en inicio de sesión' });
      }
      
      if (results.length === 0) {
        return res.status(401).json({ error: 'Credenciales incorrectas' });
      }
      
      const usuario = results[0];
      
      // Verificar contraseña
      const passwordMatch = await bcrypt.compare(password, usuario.password_hash);
      
      if (!passwordMatch) {
        // Incrementar contador de intentos fallidos
        db.query('UPDATE Autenticacion SET intentos_fallidos = intentos_fallidos + 1 WHERE id_autenticacion = ?', [usuario.id_autenticacion]);
        return res.status(401).json({ error: 'Credenciales incorrectas' });
      }
      
      // Si la cuenta no está activa
      if (!usuario.activo) {
        return res.status(403).json({ error: 'Cuenta desactivada' });
      }
      
      // Generar tokens
      const payload = {
        id: usuario.id_usuario,
        email: usuario.email,
        rol: usuario.id_rol
      };
      
      const accessToken = jwt.sign(payload, process.env.JWT_SECRET || 'clave_secreta_temporal', { expiresIn: '15m' });
      const refreshToken = jwt.sign(payload, process.env.JWT_REFRESH_SECRET || 'clave_refresco_temporal', { expiresIn: '7d' });
      
      // Actualizar último login y tokens
      const updateQuery = `
        UPDATE Autenticacion
        SET ultimo_login = NOW(),
            token_acceso = ?,
            token_refresco = ?,
            fecha_expiracion = DATE_ADD(NOW(), INTERVAL 7 DAY),
            intentos_fallidos = 0
        WHERE id_autenticacion = ?
      `;
      
      db.query(updateQuery, [accessToken, refreshToken, usuario.id_autenticacion], (err) => {
        if (err) {
          console.error('Error al actualizar datos de sesión:', err);
          return res.status(500).json({ error: 'Error en inicio de sesión' });
        }
        
        // Obtener progreso del usuario
        db.query('SELECT nivel, puntos FROM ProgresoUsuario WHERE id_usuario = ?', [usuario.id_usuario], (err, progresoResults) => {
          const progreso = progresoResults && progresoResults.length > 0 ? progresoResults[0] : { nivel: 1, puntos: 0 };
          
          // Respuesta con tokens y datos básicos
          res.json({
            id: usuario.id_usuario,
            nombre: usuario.nombre,
            email: usuario.email,
            rol: {
              id: usuario.id_rol,
              nombre: usuario.rol_nombre
            },
            nivel: progreso.nivel,
            puntos: progreso.puntos,
            accessToken,
            refreshToken
          });
        });
      });
    });
  } catch (error) {
    console.error('Error en login:', error);
    res.status(500).json({ error: 'Error en inicio de sesión' });
  }
});

// Refrescar token
router.post('/refresh-token', (req, res) => {
  const { refreshToken } = req.body;
  
  if (!refreshToken) {
    return res.status(400).json({ error: 'Token de refresco requerido' });
  }
  
  try {
    // Verificar el refresh token
    jwt.verify(refreshToken, process.env.JWT_REFRESH_SECRET || 'clave_refresco_temporal', (err, decoded) => {
      if (err) {
        return res.status(401).json({ error: 'Token de refresco inválido o expirado' });
      }
      
      // Buscar usuario por refresh token
      const query = 'SELECT a.*, u.id_usuario, u.id_rol FROM Autenticacion a JOIN Usuarios u ON a.id_autenticacion = u.id_autenticacion WHERE a.token_refresco = ?';
      db.query(query, [refreshToken], (err, results) => {
        if (err || results.length === 0) {
          return res.status(401).json({ error: 'Token de refresco inválido' });
        }
        
        const usuario = results[0];
        
        // Generar nuevo access token
        const payload = {
          id: usuario.id_usuario,
          email: usuario.email,
          rol: usuario.id_rol
        };
        
        const newAccessToken = jwt.sign(payload, process.env.JWT_SECRET || 'clave_secreta_temporal', { expiresIn: '15m' });
        
        // Actualizar token en la base de datos
        db.query('UPDATE Autenticacion SET token_acceso = ? WHERE id_autenticacion = ?', [newAccessToken, usuario.id_autenticacion]);
        
        res.json({ accessToken: newAccessToken });
      });
    });
  } catch (error) {
    console.error('Error al refrescar token:', error);
    res.status(500).json({ error: 'Error al refrescar token' });
  }
});

// Vincular robot a usuario
router.post('/vincular-robot', verificarToken, (req, res) => {
  const { codigo_vinculacion } = req.body;
  const idUsuario = req.usuario.id;
  
  if (!codigo_vinculacion) {
    return res.status(400).json({ error: 'Código de vinculación requerido' });
  }
  
  // Buscar robot con el código de vinculación
  const robotQuery = 'SELECT * FROM Robots WHERE codigo_vinculacion = ? AND id_usuario IS NULL';
  db.query(robotQuery, [codigo_vinculacion], (err, robotResults) => {
    if (err) {
      console.error('Error al buscar robot:', err);
      return res.status(500).json({ error: 'Error al vincular robot' });
    }
    
    if (robotResults.length === 0) {
      return res.status(404).json({ error: 'Robot no encontrado o ya vinculado' });
    }
    
    const robot = robotResults[0];
    
    // Registrar vinculación
    const vinculacionQuery = `
      INSERT INTO VinculacionRobots (codigo_vinculacion, id_robot, id_usuario, estado, fecha_expiracion)
      VALUES (?, ?, ?, 'activa', DATE_ADD(NOW(), INTERVAL 1 YEAR))
    `;
    
    db.query(vinculacionQuery, [codigo_vinculacion, robot.id_robot, idUsuario], (err) => {
      if (err) {
        console.error('Error al registrar vinculación:', err);
        return res.status(500).json({ error: 'Error al vincular robot' });
      }
      
      // Actualizar robot con el id de usuario
      const updateRobotQuery = 'UPDATE Robots SET id_usuario = ? WHERE id_robot = ?';
      db.query(updateRobotQuery, [idUsuario, robot.id_robot], (err) => {
        if (err) {
          console.error('Error al actualizar robot:', err);
          return res.status(500).json({ error: 'Error al vincular robot' });
        }
        
        res.json({
          mensaje: 'Robot vinculado correctamente',
          robot: {
            id: robot.id_robot,
            nombre: robot.nombre,
            modelo: robot.modelo
          }
        });
      });
    });
  });
});

// Cerrar sesión
router.post('/logout', verificarToken, (req, res) => {
  const idUsuario = req.usuario.id;
  
  // Invalidar tokens
  const query = `
    UPDATE Autenticacion a
    JOIN Usuarios u ON a.id_autenticacion = u.id_autenticacion
    SET a.token_acceso = NULL, a.token_refresco = NULL
    WHERE u.id_usuario = ?
  `;
  
  db.query(query, [idUsuario], (err) => {
    if (err) {
      console.error('Error al cerrar sesión:', err);
      return res.status(500).json({ error: 'Error al cerrar sesión' });
    }
    
    res.json({ mensaje: 'Sesión cerrada correctamente' });
  });
});

module.exports = router;
