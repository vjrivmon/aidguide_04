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

// GET /api/rutas/detalle/:id - Obtener una ruta con todos sus lugares
router.get('/detalle/:id', (req, res) => {
  // Obtener primero la ruta
  db.query('SELECT * FROM Rutas WHERE id_ruta = ?', [req.params.id], (err, rutaResults) => {
    if (err) return res.status(500).json({ error: err.message });
    if (rutaResults.length === 0) return res.status(404).json({ error: 'Ruta no encontrada' });
    
    const ruta = rutaResults[0];
    
    // Obtener los lugares de la ruta ordenados
    db.query(
      `SELECT l.*, rl.orden 
       FROM Lugares l
       JOIN RutaLugares rl ON l.id_lugar = rl.id_lugar
       WHERE rl.id_ruta = ?
       ORDER BY rl.orden ASC`,
      [req.params.id],
      (err, lugaresResults) => {
        if (err) return res.status(500).json({ error: err.message });
        
        res.json({
          ...ruta,
          lugares: lugaresResults
        });
      }
    );
  });
});

// POST /api/rutas - Crear una nueva ruta
router.post('/', (req, res) => {
  const { 
    fecha, duracion, mapa, ultimo_uso, descripcion,
    completada, id_usuario, lugares
  } = req.body;
  
  if (!id_usuario) {
    return res.status(400).json({ error: 'El usuario es obligatorio' });
  }
  
  if (!lugares || !Array.isArray(lugares) || lugares.length < 2) {
    return res.status(400).json({ error: 'Una ruta debe tener al menos dos lugares (origen y destino)' });
  }
  
  const now = new Date();
  
  // Iniciamos una transacción para asegurar la integridad de los datos
  db.beginTransaction(err => {
    if (err) return res.status(500).json({ error: err.message });
    
    // Insertar primero la ruta
    const rutaQuery = `
      INSERT INTO Rutas 
      (fecha, duracion, mapa, ultimo_uso, descripcion, completada, id_usuario) 
      VALUES (?, ?, ?, ?, ?, ?, ?)`;
    
    const rutaValues = [
      fecha || now, duracion || 0, mapa || null, 
      ultimo_uso || now, descripcion || null,
      completada !== undefined ? completada : false, id_usuario
    ];
    
    db.query(rutaQuery, rutaValues, (err, rutaResult) => {
      if (err) {
        return db.rollback(() => {
          res.status(500).json({ error: err.message });
        });
      }
      
      const id_ruta = rutaResult.insertId;
      
      // Insertar los lugares de la ruta
      const rutaLugaresQuery = `
        INSERT INTO RutaLugares (id_ruta, id_lugar, orden)
        VALUES (?, ?, ?)`;
      
      const insertPromises = lugares.map((lugar, index) => {
        return new Promise((resolve, reject) => {
          db.query(rutaLugaresQuery, [id_ruta, lugar, index], (err, result) => {
            if (err) return reject(err);
            resolve(result);
          });
        });
      });
      
      Promise.all(insertPromises)
        .then(() => {
          // Confirmar la transacción
          db.commit(err => {
            if (err) {
              return db.rollback(() => {
                res.status(500).json({ error: err.message });
              });
            }
            
            // Devolver la ruta creada con sus lugares
            res.status(201).json({
              id_ruta,
              fecha: fecha || now,
              duracion: duracion || 0,
              mapa,
              ultimo_uso: ultimo_uso || now,
              descripcion,
              completada: completada !== undefined ? completada : false,
              id_usuario,
              lugares: lugares.map((lugar, index) => ({ id_lugar: lugar, orden: index }))
            });
          });
        })
        .catch(err => {
          db.rollback(() => {
            res.status(500).json({ error: err.message });
          });
        });
    });
  });
});

// PUT /api/rutas/:id - Actualizar una ruta
router.put('/:id', (req, res) => {
  const { 
    fecha, duracion, mapa, ultimo_uso, descripcion, 
    completada, id_usuario, lugares
  } = req.body;
  
  // Primero verificamos si la ruta existe
  db.query('SELECT * FROM Rutas WHERE id_ruta = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Ruta no encontrada' });
    
    const ruta = results[0];
    const id_ruta = parseInt(req.params.id);
    
    // Preparamos los valores actualizados
    const updatedRoute = {
      fecha: fecha || ruta.fecha,
      duracion: duracion !== undefined ? duracion : ruta.duracion,
      mapa: mapa || ruta.mapa,
      ultimo_uso: ultimo_uso || new Date(),
      descripcion: descripcion || ruta.descripcion,
      completada: completada !== undefined ? completada : ruta.completada,
      id_usuario: id_usuario || ruta.id_usuario
    };
    
    // Iniciamos una transacción
    db.beginTransaction(err => {
      if (err) return res.status(500).json({ error: err.message });
      
      // Actualizar la ruta
      const query = `
        UPDATE Rutas 
        SET fecha = ?, duracion = ?, mapa = ?, ultimo_uso = ?, 
            descripcion = ?, completada = ?, id_usuario = ?
        WHERE id_ruta = ?`;
      
      const values = [
        updatedRoute.fecha, updatedRoute.duracion, updatedRoute.mapa, 
        updatedRoute.ultimo_uso, updatedRoute.descripcion, 
        updatedRoute.completada, updatedRoute.id_usuario, id_ruta
      ];
      
      db.query(query, values, (err, result) => {
        if (err) {
          return db.rollback(() => {
            res.status(500).json({ error: err.message });
          });
        }
        
        // Si no se proporcionan lugares, finalizamos
        if (!lugares || !Array.isArray(lugares)) {
          return db.commit(err => {
            if (err) {
              return db.rollback(() => {
                res.status(500).json({ error: err.message });
              });
            }
            res.json({
              id_ruta,
              ...updatedRoute
            });
          });
        }
        
        // Si hay lugares, primero eliminamos los existentes
        db.query('DELETE FROM RutaLugares WHERE id_ruta = ?', [id_ruta], (err, result) => {
          if (err) {
            return db.rollback(() => {
              res.status(500).json({ error: err.message });
            });
          }
          
          // Luego insertamos los nuevos lugares
          const rutaLugaresQuery = `
            INSERT INTO RutaLugares (id_ruta, id_lugar, orden)
            VALUES (?, ?, ?)`;
          
          const insertPromises = lugares.map((lugar, index) => {
            return new Promise((resolve, reject) => {
              db.query(rutaLugaresQuery, [id_ruta, lugar, index], (err, result) => {
                if (err) return reject(err);
                resolve(result);
              });
            });
          });
          
          Promise.all(insertPromises)
            .then(() => {
              // Confirmar la transacción
              db.commit(err => {
                if (err) {
                  return db.rollback(() => {
                    res.status(500).json({ error: err.message });
                  });
                }
                
                // Devolver la ruta actualizada con sus lugares
                res.json({
                  id_ruta,
                  ...updatedRoute,
                  lugares: lugares.map((lugar, index) => ({ id_lugar: lugar, orden: index }))
                });
              });
            })
            .catch(err => {
              db.rollback(() => {
                res.status(500).json({ error: err.message });
              });
            });
        });
      });
    });
  });
});

// DELETE /api/rutas/:id - Eliminar una ruta
router.delete('/:id', (req, res) => {
  const id_ruta = req.params.id;
  
  // Iniciamos una transacción
  db.beginTransaction(err => {
    if (err) return res.status(500).json({ error: err.message });
    
    // Primero eliminamos los lugares asociados a la ruta
    db.query('DELETE FROM RutaLugares WHERE id_ruta = ?', [id_ruta], (err, result) => {
      if (err) {
        return db.rollback(() => {
          res.status(500).json({ error: err.message });
        });
      }
      
      // Después eliminamos la ruta
      db.query('DELETE FROM Rutas WHERE id_ruta = ?', [id_ruta], (err, result) => {
        if (err) {
          return db.rollback(() => {
            res.status(500).json({ error: err.message });
          });
        }
        
        if (result.affectedRows === 0) {
          return db.rollback(() => {
            res.status(404).json({ error: 'Ruta no encontrada' });
          });
        }
        
        // Confirmamos la transacción
        db.commit(err => {
          if (err) {
            return db.rollback(() => {
              res.status(500).json({ error: err.message });
            });
          }
          
          res.json({ message: 'Ruta eliminada con éxito' });
        });
      });
    });
  });
});

module.exports = router; 