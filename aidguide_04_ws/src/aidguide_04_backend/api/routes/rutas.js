const express = require('express');
const router = express.Router();
const db = require('../db');

// GET /api/rutas - Obtener todas las rutas
router.get('/', (req, res) => {
  db.query('SELECT * FROM Rutas', (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/rutas/:id - Obtener una ruta por ID
router.get('/:id', (req, res) => {
  db.query('SELECT * FROM Rutas WHERE id_ruta = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Ruta no encontrada' });
    res.json(results[0]);
  });
});

// GET /api/rutas/usuario/:id - Obtener rutas de un usuario
router.get('/usuario/:id', (req, res) => {
  db.query('SELECT * FROM Rutas WHERE id_usuario = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/rutas/completadas/:id - Obtener rutas completadas de un usuario
router.get('/completadas/:id', (req, res) => {
  db.query('SELECT * FROM Rutas WHERE id_usuario = ? AND completada = true', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// POST /api/rutas - Crear una nueva ruta
router.post('/', (req, res) => {
  const { 
    fecha, id_origen, id_destino, duracion, mapa, 
    ultimo_uso, descripcion, completada, id_usuario 
  } = req.body;
  
  if (!id_origen || !id_destino || !id_usuario) {
    return res.status(400).json({ error: 'Origen, destino y usuario son obligatorios' });
  }
  
  const query = `
    INSERT INTO Rutas 
    (fecha, id_origen, id_destino, duracion, mapa, ultimo_uso, descripcion, completada, id_usuario) 
    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)`;
  
  const now = new Date();
  
  const values = [
    fecha || now, id_origen, id_destino, duracion || 0, 
    mapa || null, ultimo_uso || now, descripcion || null, 
    completada !== undefined ? completada : false, id_usuario
  ];
  
  db.query(query, values, (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    
    res.status(201).json({
      id_ruta: result.insertId,
      fecha: fecha || now,
      id_origen,
      id_destino,
      duracion: duracion || 0,
      mapa,
      ultimo_uso: ultimo_uso || now,
      descripcion,
      completada: completada !== undefined ? completada : false,
      id_usuario
    });
  });
});

// PUT /api/rutas/:id - Actualizar una ruta
router.put('/:id', (req, res) => {
  const { 
    fecha, id_origen, id_destino, duracion, mapa, 
    ultimo_uso, descripcion, completada, id_usuario 
  } = req.body;
  
  // Primero verificamos si la ruta existe
  db.query('SELECT * FROM Rutas WHERE id_ruta = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Ruta no encontrada' });
    
    const ruta = results[0];
    
    // Preparamos los valores actualizados
    const updatedRoute = {
      fecha: fecha || ruta.fecha,
      id_origen: id_origen || ruta.id_origen,
      id_destino: id_destino || ruta.id_destino,
      duracion: duracion !== undefined ? duracion : ruta.duracion,
      mapa: mapa || ruta.mapa,
      ultimo_uso: ultimo_uso || new Date(),  // Actualizamos el último uso por defecto
      descripcion: descripcion || ruta.descripcion,
      completada: completada !== undefined ? completada : ruta.completada,
      id_usuario: id_usuario || ruta.id_usuario
    };
    
    const query = `
      UPDATE Rutas 
      SET fecha = ?, id_origen = ?, id_destino = ?, duracion = ?, 
          mapa = ?, ultimo_uso = ?, descripcion = ?, completada = ?, id_usuario = ?
      WHERE id_ruta = ?`;
    
    const values = [
      updatedRoute.fecha, updatedRoute.id_origen, updatedRoute.id_destino,
      updatedRoute.duracion, updatedRoute.mapa, updatedRoute.ultimo_uso,
      updatedRoute.descripcion, updatedRoute.completada, updatedRoute.id_usuario,
      req.params.id
    ];
    
    db.query(query, values, (err, result) => {
      if (err) return res.status(500).json({ error: err.message });
      
      res.json({
        id_ruta: parseInt(req.params.id),
        ...updatedRoute
      });
    });
  });
});

// DELETE /api/rutas/:id - Eliminar una ruta
router.delete('/:id', (req, res) => {
  db.query('DELETE FROM Rutas WHERE id_ruta = ?', [req.params.id], (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    if (result.affectedRows === 0) return res.status(404).json({ error: 'Ruta no encontrada' });
    res.json({ message: 'Ruta eliminada con éxito' });
  });
});

module.exports = router; 