const express = require('express');
const router = express.Router();
const db = require('../db');

// GET /api/robots - Obtener todos los robots
router.get('/', (req, res) => {
  db.query('SELECT * FROM Robots', (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/robots/:id - Obtener un robot por ID
router.get('/:id', (req, res) => {
  db.query('SELECT * FROM Robots WHERE id_robot = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Robot no encontrado' });
    res.json(results[0]);
  });
});

// GET /api/robots/usuario/:id - Obtener robots de un usuario
router.get('/usuario/:id', (req, res) => {
  db.query('SELECT * FROM Robots WHERE id_usuario = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
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

// DELETE /api/robots/:id - Eliminar un robot
router.delete('/:id', (req, res) => {
  db.query('DELETE FROM Robots WHERE id_robot = ?', [req.params.id], (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    if (result.affectedRows === 0) return res.status(404).json({ error: 'Robot no encontrado' });
    res.json({ message: 'Robot eliminado con éxito' });
  });
});

module.exports = router; 