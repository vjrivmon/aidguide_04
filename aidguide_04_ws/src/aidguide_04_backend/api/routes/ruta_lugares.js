const express = require('express');
const router = express.Router();
const db = require('../db');

// GET /api/ruta-lugares - Obtener todas las relaciones ruta-lugar
router.get('/', (req, res) => {
  db.query('SELECT * FROM RutaLugares ORDER BY id_ruta, orden', (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/ruta-lugares/ruta/:id - Obtener todos los lugares de una ruta en orden
router.get('/ruta/:id', (req, res) => {
  const query = `
    SELECT rl.*, l.nombre, l.direccion, l.latitud, l.longitud 
    FROM RutaLugares rl
    JOIN Lugares l ON rl.id_lugar = l.id_lugar
    WHERE rl.id_ruta = ?
    ORDER BY rl.orden`;
  
  db.query(query, [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/ruta-lugares/lugar/:id - Obtener todas las rutas que pasan por un lugar
router.get('/lugar/:id', (req, res) => {
  const query = `
    SELECT rl.*, r.descripcion, r.fecha
    FROM RutaLugares rl
    JOIN Rutas r ON rl.id_ruta = r.id_ruta
    WHERE rl.id_lugar = ?
    ORDER BY r.fecha DESC`;
  
  db.query(query, [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// POST /api/ruta-lugares - Añadir un lugar a una ruta
router.post('/', (req, res) => {
  const { id_ruta, id_lugar, orden } = req.body;
  
  if (!id_ruta || !id_lugar || orden === undefined) {
    return res.status(400).json({ error: 'Todos los campos son obligatorios' });
  }
  
  // Comprobar si la ruta y el lugar existen
  const checkQuery = `
    SELECT 
      (SELECT COUNT(*) FROM Rutas WHERE id_ruta = ?) as rutaExists,
      (SELECT COUNT(*) FROM Lugares WHERE id_lugar = ?) as lugarExists`;
  
  db.query(checkQuery, [id_ruta, id_lugar], (err, checkResults) => {
    if (err) return res.status(500).json({ error: err.message });
    
    const { rutaExists, lugarExists } = checkResults[0];
    
    if (rutaExists === 0) {
      return res.status(404).json({ error: 'La ruta no existe' });
    }
    
    if (lugarExists === 0) {
      return res.status(404).json({ error: 'El lugar no existe' });
    }
    
    // Comprobar si ya existe esta relación
    db.query(
      'SELECT * FROM RutaLugares WHERE id_ruta = ? AND id_lugar = ?',
      [id_ruta, id_lugar],
      (err, existingResults) => {
        if (err) return res.status(500).json({ error: err.message });
        
        if (existingResults.length > 0) {
          return res.status(400).json({ error: 'Este lugar ya está asociado a esta ruta' });
        }
        
        // Insertar la nueva relación
        db.query(
          'INSERT INTO RutaLugares (id_ruta, id_lugar, orden) VALUES (?, ?, ?)',
          [id_ruta, id_lugar, orden],
          (err, result) => {
            if (err) return res.status(500).json({ error: err.message });
            
            res.status(201).json({
              id_ruta,
              id_lugar,
              orden
            });
          }
        );
      }
    );
  });
});

// PUT /api/ruta-lugares - Actualizar el orden de un lugar en una ruta
router.put('/', (req, res) => {
  const { id_ruta, id_lugar, orden } = req.body;
  
  if (!id_ruta || !id_lugar || orden === undefined) {
    return res.status(400).json({ error: 'Todos los campos son obligatorios' });
  }
  
  // Comprobar si existe la relación
  db.query(
    'SELECT * FROM RutaLugares WHERE id_ruta = ? AND id_lugar = ?',
    [id_ruta, id_lugar],
    (err, results) => {
      if (err) return res.status(500).json({ error: err.message });
      
      if (results.length === 0) {
        return res.status(404).json({ error: 'Relación ruta-lugar no encontrada' });
      }
      
      // Actualizar el orden
      db.query(
        'UPDATE RutaLugares SET orden = ? WHERE id_ruta = ? AND id_lugar = ?',
        [orden, id_ruta, id_lugar],
        (err, result) => {
          if (err) return res.status(500).json({ error: err.message });
          
          res.json({
            id_ruta,
            id_lugar,
            orden
          });
        }
      );
    }
  );
});

// DELETE /api/ruta-lugares - Eliminar un lugar de una ruta
router.delete('/', (req, res) => {
  const { id_ruta, id_lugar } = req.body;
  
  if (!id_ruta || !id_lugar) {
    return res.status(400).json({ error: 'Ruta y lugar son obligatorios' });
  }
  
  db.query(
    'DELETE FROM RutaLugares WHERE id_ruta = ? AND id_lugar = ?',
    [id_ruta, id_lugar],
    (err, result) => {
      if (err) return res.status(500).json({ error: err.message });
      
      if (result.affectedRows === 0) {
        return res.status(404).json({ error: 'Relación ruta-lugar no encontrada' });
      }
      
      res.json({ message: 'Lugar eliminado de la ruta con éxito' });
    }
  );
});

module.exports = router; 