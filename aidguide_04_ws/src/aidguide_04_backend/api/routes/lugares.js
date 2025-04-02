const express = require('express');
const router = express.Router();
const db = require('../db');

// GET /api/lugares - Obtener todos los lugares
router.get('/', (req, res) => {
  db.query('SELECT * FROM Lugares', (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/lugares/:id - Obtener un lugar por ID
router.get('/:id', (req, res) => {
  db.query('SELECT * FROM Lugares WHERE id_lugar = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Lugar no encontrado' });
    res.json(results[0]);
  });
});

// GET /api/lugares/usuario/:id - Obtener lugares de un usuario
router.get('/usuario/:id', (req, res) => {
  db.query('SELECT * FROM Lugares WHERE id_usuario = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/lugares/favoritos/:id - Obtener lugares favoritos de un usuario
router.get('/favoritos/:id', (req, res) => {
  db.query('SELECT * FROM Lugares WHERE id_usuario = ? AND favorito = true', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// POST /api/lugares - Crear un nuevo lugar
router.post('/', (req, res) => {
  const { 
    id_usuario, nombre, direccion, favorito, latitud, longitud 
  } = req.body;
  
  if (!id_usuario || !nombre || !direccion) {
    return res.status(400).json({ error: 'Usuario, nombre y dirección son obligatorios' });
  }
  
  const query = `
    INSERT INTO Lugares 
    (id_usuario, nombre, direccion, favorito, latitud, longitud) 
    VALUES (?, ?, ?, ?, ?, ?)`;
  
  const values = [
    id_usuario, nombre, direccion, favorito || false, latitud || null, longitud || null
  ];
  
  db.query(query, values, (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    
    res.status(201).json({
      id_lugar: result.insertId,
      id_usuario,
      nombre,
      direccion,
      favorito: favorito || false,
      latitud,
      longitud
    });
  });
});

// PUT /api/lugares/:id - Actualizar un lugar
router.put('/:id', (req, res) => {
  const { 
    id_usuario, nombre, direccion, favorito, latitud, longitud 
  } = req.body;
  
  // Primero verificamos si el lugar existe
  db.query('SELECT * FROM Lugares WHERE id_lugar = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Lugar no encontrado' });
    
    const lugar = results[0];
    
    // Preparamos los valores actualizados
    const updatedPlace = {
      id_usuario: id_usuario || lugar.id_usuario,
      nombre: nombre || lugar.nombre,
      direccion: direccion || lugar.direccion,
      favorito: favorito !== undefined ? favorito : lugar.favorito,
      latitud: latitud !== undefined ? latitud : lugar.latitud,
      longitud: longitud !== undefined ? longitud : lugar.longitud
    };
    
    const query = `
      UPDATE Lugares 
      SET id_usuario = ?, nombre = ?, direccion = ?, favorito = ?, latitud = ?, longitud = ?
      WHERE id_lugar = ?`;
    
    const values = [
      updatedPlace.id_usuario, updatedPlace.nombre, updatedPlace.direccion,
      updatedPlace.favorito, updatedPlace.latitud, updatedPlace.longitud,
      req.params.id
    ];
    
    db.query(query, values, (err, result) => {
      if (err) return res.status(500).json({ error: err.message });
      
      res.json({
        id_lugar: parseInt(req.params.id),
        ...updatedPlace
      });
    });
  });
});

// DELETE /api/lugares/:id - Eliminar un lugar
router.delete('/:id', (req, res) => {
  db.query('DELETE FROM Lugares WHERE id_lugar = ?', [req.params.id], (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    if (result.affectedRows === 0) return res.status(404).json({ error: 'Lugar no encontrado' });
    res.json({ message: 'Lugar eliminado con éxito' });
  });
});

module.exports = router; 