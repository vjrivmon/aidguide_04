const express = require('express');
const router = express.Router();
const db = require('../db');
const { v4: uuidv4 } = require('uuid');

// Middleware para verificar token de robot (si aplicable)
const verificarTokenRobot = (req, res, next) => {
  const token = req.headers['robot-token'];
  
  if (!token) {
    return res.status(403).json({ error: 'No se proporcionó token de robot' });
  }
  
  // Verificar token en la base de datos
  const query = 'SELECT * FROM Robots WHERE token_acceso = ?';
  db.query(query, [token], (err, results) => {
    if (err || results.length === 0) {
      return res.status(401).json({ error: 'Token de robot inválido' });
    }
    
    // Adjuntar información del robot a la solicitud
    req.robot = results[0];
    next();
  });
};

// GET /api/robots - Obtener todos los robots
router.get('/', (req, res) => {
  const query = 'SELECT * FROM Robots';
  db.query(query, (err, results) => {
    if (err) {
      console.error('Error al obtener robots:', err);
      return res.status(500).json({ error: 'Error al obtener robots' });
    }
    res.json(results);
  });
});

// GET /api/robots/:id - Obtener un robot por ID
router.get('/:id', (req, res) => {
  const { id } = req.params;
  const query = 'SELECT * FROM Robots WHERE id_robot = ?';
  
  db.query(query, [id], (err, results) => {
    if (err) {
      console.error('Error al obtener robot:', err);
      return res.status(500).json({ error: 'Error al obtener robot' });
    }
    
    if (results.length === 0) {
      return res.status(404).json({ error: 'Robot no encontrado' });
    }
    
    res.json(results[0]);
  });
});

// GET /api/robots/usuario/:id - Obtener robots de un usuario
router.get('/usuario/:id', (req, res) => {
  const { id } = req.params;
  const query = 'SELECT * FROM Robots WHERE id_usuario = ?';
  
  db.query(query, [id], (err, results) => {
    if (err) {
      console.error('Error al obtener robots del usuario:', err);
      return res.status(500).json({ error: 'Error al obtener robots' });
    }
    
    res.json(results);
  });
});

// Registrar un nuevo robot
router.post('/registro', (req, res) => {
  const { nombre, modelo, serie } = req.body;
  
  if (!nombre || !modelo || !serie) {
    return res.status(400).json({ error: 'Todos los campos son obligatorios' });
  }
  
  // Verificar si ya existe un robot con la misma serie
  const checkQuery = 'SELECT * FROM Robots WHERE serie = ?';
  db.query(checkQuery, [serie], (err, results) => {
    if (err) {
      console.error('Error al verificar robot:', err);
      return res.status(500).json({ error: 'Error al registrar robot' });
    }
    
    if (results.length > 0) {
      return res.status(400).json({ error: 'Ya existe un robot con este número de serie' });
    }
    
    // Generar código de vinculación y token de acceso
    const codigoVinculacion = Math.floor(1000000000 + Math.random() * 9000000000).toString().substring(0, 10);
    const tokenAcceso = uuidv4();
    
    // Insertar nuevo robot
    const query = `
      INSERT INTO Robots (nombre, modelo, serie, codigo_vinculacion, token_acceso, activo)
      VALUES (?, ?, ?, ?, ?, true)
    `;
    
    db.query(query, [nombre, modelo, serie, codigoVinculacion, tokenAcceso], (err, result) => {
      if (err) {
        console.error('Error al registrar robot:', err);
        return res.status(500).json({ error: 'Error al registrar robot' });
      }
      
      res.status(201).json({
        mensaje: 'Robot registrado correctamente',
        robot: {
          id: result.insertId,
          nombre,
          modelo,
          serie,
          codigo_vinculacion: codigoVinculacion,
          token_acceso: tokenAcceso
        }
      });
    });
  });
});

// POST /api/robots - Crear un nuevo robot
router.post('/', (req, res) => {
  const { 
    nombre, modelo, version_firmware, bateria, conexion, estado,
    averias, ubicacion, velocidad_navegacion, id_usuario 
  } = req.body;
  
  if (!nombre || !modelo || !id_usuario) {
    return res.status(400).json({ error: 'Nombre, modelo y usuario son obligatorios' });
  }
  
  const query = `
    INSERT INTO Robots 
    (nombre, modelo, version_firmware, bateria, conexion, estado, averias, ubicacion, velocidad_navegacion, id_usuario) 
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)`;
  
  const values = [
    nombre, modelo, version_firmware || null, bateria || 100, 
    conexion || 'Desconectado', estado || 'Inactivo', averias || null, 
    ubicacion || null, velocidad_navegacion || 'Normal', id_usuario
  ];
  
  db.query(query, values, (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    
    res.status(201).json({
      id_robot: result.insertId,
      nombre,
      modelo,
      version_firmware,
      bateria: bateria || 100,
      conexion: conexion || 'Desconectado',
      estado: estado || 'Inactivo',
      averias,
      ubicacion,
      velocidad_navegacion: velocidad_navegacion || 'Normal',
      id_usuario
    });
  });
});

// PUT /api/robots/:id - Actualizar un robot
router.put('/:id', (req, res) => {
  const { 
    nombre, modelo, version_firmware, bateria, conexion, estado,
    averias, ubicacion, velocidad_navegacion, id_usuario 
  } = req.body;
  
  // Primero verificamos si el robot existe
  db.query('SELECT * FROM Robots WHERE id_robot = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Robot no encontrado' });
    
    const robot = results[0];
    
    // Preparamos los valores actualizados
    const updatedRobot = {
      nombre: nombre || robot.nombre,
      modelo: modelo || robot.modelo,
      version_firmware: version_firmware || robot.version_firmware,
      bateria: bateria !== undefined ? bateria : robot.bateria,
      conexion: conexion || robot.conexion,
      estado: estado || robot.estado,
      averias: averias || robot.averias,
      ubicacion: ubicacion || robot.ubicacion,
      velocidad_navegacion: velocidad_navegacion || robot.velocidad_navegacion,
      id_usuario: id_usuario || robot.id_usuario
    };
    
    const query = `
      UPDATE Robots 
      SET nombre = ?, modelo = ?, version_firmware = ?, bateria = ?, 
          conexion = ?, estado = ?, averias = ?, ubicacion = ?, 
          velocidad_navegacion = ?, id_usuario = ?
      WHERE id_robot = ?`;
    
    const values = [
      updatedRobot.nombre, updatedRobot.modelo, updatedRobot.version_firmware,
      updatedRobot.bateria, updatedRobot.conexion, updatedRobot.estado,
      updatedRobot.averias, updatedRobot.ubicacion, updatedRobot.velocidad_navegacion,
      updatedRobot.id_usuario, req.params.id
    ];
    
    db.query(query, values, (err, result) => {
      if (err) return res.status(500).json({ error: err.message });
      
      res.json({
        id_robot: parseInt(req.params.id),
        ...updatedRobot
      });
    });
  });
});

// Actualizar estado del robot
router.put('/:id/estado', verificarTokenRobot, (req, res) => {
  const { id } = req.params;
  const { estado, ubicacion, bateria, ultima_conexion } = req.body;
  
  // El robot solo puede actualizar su propio estado
  if (parseInt(id) !== req.robot.id_robot) {
    return res.status(403).json({ error: 'No autorizado para actualizar este robot' });
  }
  
  let updates = [];
  let values = [];
  
  if (estado) {
    updates.push('estado = ?');
    values.push(estado);
  }
  
  if (ubicacion) {
    updates.push('ubicacion = ?');
    values.push(ubicacion);
  }
  
  if (bateria) {
    updates.push('bateria = ?');
    values.push(bateria);
  }
  
  if (ultima_conexion) {
    updates.push('ultima_conexion = ?');
    values.push(ultima_conexion);
  } else {
    updates.push('ultima_conexion = NOW()');
  }
  
  if (updates.length === 0) {
    return res.status(400).json({ error: 'No se proporcionaron datos para actualizar' });
  }
  
  const query = `UPDATE Robots SET ${updates.join(', ')} WHERE id_robot = ?`;
  values.push(id);
  
  db.query(query, values, (err, result) => {
    if (err) {
      console.error('Error al actualizar estado del robot:', err);
      return res.status(500).json({ error: 'Error al actualizar estado' });
    }
    
    if (result.affectedRows === 0) {
      return res.status(404).json({ error: 'Robot no encontrado' });
    }
    
    res.json({ mensaje: 'Estado del robot actualizado correctamente' });
  });
});

// Desvincular robot de usuario
router.post('/:id/desvincular', (req, res) => {
  const { id } = req.params;
  
  // Actualizar la vinculación
  const updateVinculacion = `
    UPDATE VinculacionRobots
    SET estado = 'inactiva'
    WHERE id_robot = ? AND estado = 'activa'
  `;
  
  db.query(updateVinculacion, [id], (err, vinculacionResult) => {
    if (err) {
      console.error('Error al desvincular robot:', err);
      return res.status(500).json({ error: 'Error al desvincular robot' });
    }
    
    // Actualizar el robot
    const updateRobot = 'UPDATE Robots SET id_usuario = NULL WHERE id_robot = ?';
    db.query(updateRobot, [id], (err, robotResult) => {
      if (err) {
        console.error('Error al actualizar robot:', err);
        return res.status(500).json({ error: 'Error al desvincular robot' });
      }
      
      if (robotResult.affectedRows === 0) {
        return res.status(404).json({ error: 'Robot no encontrado' });
      }
      
      res.json({ mensaje: 'Robot desvinculado correctamente' });
    });
  });
});

// Eliminar un robot
router.delete('/:id', (req, res) => {
  const { id } = req.params;
  
  // Primero desactivamos, no eliminamos físicamente
  const query = 'UPDATE Robots SET activo = false WHERE id_robot = ?';
  
  db.query(query, [id], (err, result) => {
    if (err) {
      console.error('Error al eliminar robot:', err);
      return res.status(500).json({ error: 'Error al eliminar robot' });
    }
    
    if (result.affectedRows === 0) {
      return res.status(404).json({ error: 'Robot no encontrado' });
    }
    
    res.json({ mensaje: 'Robot eliminado correctamente' });
  });
});

module.exports = router; 