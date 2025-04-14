const express = require('express');
const router = express.Router();
const db = require('../db');

// GET /api/recompensas - Obtener todas las recompensas
router.get('/', (req, res) => {
  db.query('SELECT * FROM Recompensas', (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/recompensas/:id - Obtener una recompensa por ID
router.get('/:id', (req, res) => {
  db.query('SELECT * FROM Recompensas WHERE id_recompensa = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Recompensa no encontrada' });
    res.json(results[0]);
  });
});

// GET /api/recompensas/tipo/:tipo - Obtener recompensas por tipo
router.get('/tipo/:tipo', (req, res) => {
  const tipos = ['diario', 'semanal', 'mensual', 'logro'];
  
  if (!tipos.includes(req.params.tipo)) {
    return res.status(400).json({ error: 'Tipo de recompensa inválido' });
  }
  
  db.query('SELECT * FROM Recompensas WHERE tipo = ?', [req.params.tipo], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/recompensas/nivel/:nivel - Obtener recompensas por nivel requerido
router.get('/nivel/:nivel', (req, res) => {
  db.query('SELECT * FROM Recompensas WHERE nivel_requerido <= ?', [req.params.nivel], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/recompensas/usuario/:id - Obtener recompensas obtenidas por un usuario
router.get('/usuario/:id', (req, res) => {
  const query = `
    SELECT r.*, ro.fecha_obtencion, ro.puntos_obtenidos, ro.progreso, ro.completada
    FROM Recompensas r
    JOIN RecompensasObtenidas ro ON r.id_recompensa = ro.id_recompensa
    WHERE ro.id_usuario = ?`;
  
  db.query(query, [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/recompensas/disponibles/:id - Obtener recompensas disponibles para un usuario
router.get('/disponibles/:id', (req, res) => {
  // Primero obtenemos el nivel del usuario
  db.query('SELECT nivel FROM ProgresoUsuario WHERE id_usuario = ?', [req.params.id], (err, nivelResults) => {
    if (err) return res.status(500).json({ error: err.message });
    
    if (nivelResults.length === 0) {
      return res.status(404).json({ error: 'Usuario no encontrado o sin progreso registrado' });
    }
    
    const nivel = nivelResults[0].nivel;
    
    // Obtenemos las recompensas que el usuario no ha obtenido y que están disponibles para su nivel
    const query = `
      SELECT r.*
      FROM Recompensas r
      WHERE r.nivel_requerido <= ?
      AND r.id_recompensa NOT IN (
        SELECT ro.id_recompensa FROM RecompensasObtenidas ro WHERE ro.id_usuario = ? AND ro.completada = true
      )`;
    
    db.query(query, [nivel, req.params.id], (err, results) => {
      if (err) return res.status(500).json({ error: err.message });
      res.json(results);
    });
  });
});

// POST /api/recompensas - Crear una nueva recompensa
router.post('/', (req, res) => {
  const { 
    nombre, descripcion, puntos_requeridos, 
    nivel_requerido, tipo, icono 
  } = req.body;
  
  if (!nombre || !descripcion || puntos_requeridos === undefined || nivel_requerido === undefined) {
    return res.status(400).json({ error: 'Nombre, descripción, puntos requeridos y nivel requerido son obligatorios' });
  }
  
  const query = `
    INSERT INTO Recompensas (nombre, descripcion, puntos_requeridos, nivel_requerido, tipo, icono)
    VALUES (?, ?, ?, ?, ?, ?)`;
  
  db.query(
    query, 
    [nombre, descripcion, puntos_requeridos, nivel_requerido, tipo || 'logro', icono || null], 
    (err, result) => {
      if (err) return res.status(500).json({ error: err.message });
      
      res.status(201).json({
        id_recompensa: result.insertId,
        nombre,
        descripcion,
        puntos_requeridos,
        nivel_requerido,
        tipo: tipo || 'logro',
        icono
      });
    }
  );
});

// PUT /api/recompensas/:id - Actualizar una recompensa
router.put('/:id', (req, res) => {
  const { 
    nombre, descripcion, puntos_requeridos, 
    nivel_requerido, tipo, icono 
  } = req.body;
  
  // Verificar si existe la recompensa
  db.query('SELECT * FROM Recompensas WHERE id_recompensa = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Recompensa no encontrada' });
    
    const recompensa = results[0];
    
    // Preparar los datos actualizados
    const updatedRecompensa = {
      nombre: nombre || recompensa.nombre,
      descripcion: descripcion || recompensa.descripcion,
      puntos_requeridos: puntos_requeridos !== undefined ? puntos_requeridos : recompensa.puntos_requeridos,
      nivel_requerido: nivel_requerido !== undefined ? nivel_requerido : recompensa.nivel_requerido,
      tipo: tipo || recompensa.tipo,
      icono: icono !== undefined ? icono : recompensa.icono
    };
    
    const query = `
      UPDATE Recompensas 
      SET nombre = ?, descripcion = ?, puntos_requeridos = ?, 
          nivel_requerido = ?, tipo = ?, icono = ?
      WHERE id_recompensa = ?`;
    
    db.query(
      query, 
      [
        updatedRecompensa.nombre, updatedRecompensa.descripcion, updatedRecompensa.puntos_requeridos,
        updatedRecompensa.nivel_requerido, updatedRecompensa.tipo, updatedRecompensa.icono,
        req.params.id
      ], 
      (err, result) => {
        if (err) return res.status(500).json({ error: err.message });
        
        res.json({
          id_recompensa: parseInt(req.params.id),
          ...updatedRecompensa
        });
      }
    );
  });
});

// DELETE /api/recompensas/:id - Eliminar una recompensa
router.delete('/:id', (req, res) => {
  // Primero comprobamos si la recompensa tiene usuarios asociados
  db.query(
    'SELECT COUNT(*) as count FROM RecompensasObtenidas WHERE id_recompensa = ?',
    [req.params.id],
    (err, countResults) => {
      if (err) return res.status(500).json({ error: err.message });
      
      if (countResults[0].count > 0) {
        return res.status(400).json({ 
          error: 'No se puede eliminar la recompensa porque tiene usuarios asociados' 
        });
      }
      
      // Eliminamos la recompensa
      db.query('DELETE FROM Recompensas WHERE id_recompensa = ?', [req.params.id], (err, result) => {
        if (err) return res.status(500).json({ error: err.message });
        if (result.affectedRows === 0) return res.status(404).json({ error: 'Recompensa no encontrada' });
        
        res.json({ message: 'Recompensa eliminada con éxito' });
      });
    }
  );
});

// POST /api/recompensas/asignar - Asignar una recompensa a un usuario
router.post('/asignar', (req, res) => {
  const { id_usuario, id_recompensa, puntos_obtenidos, progreso, completada } = req.body;
  
  if (!id_usuario || !id_recompensa) {
    return res.status(400).json({ error: 'Usuario y recompensa son obligatorios' });
  }
  
  // Verificamos si existen el usuario y la recompensa
  const checkQuery = `
    SELECT 
      (SELECT COUNT(*) FROM Usuarios WHERE id_usuario = ?) as usuarioExists,
      (SELECT COUNT(*) FROM Recompensas WHERE id_recompensa = ?) as recompensaExists`;
  
  db.query(checkQuery, [id_usuario, id_recompensa], (err, checkResults) => {
    if (err) return res.status(500).json({ error: err.message });
    
    const { usuarioExists, recompensaExists } = checkResults[0];
    
    if (usuarioExists === 0) {
      return res.status(404).json({ error: 'El usuario no existe' });
    }
    
    if (recompensaExists === 0) {
      return res.status(404).json({ error: 'La recompensa no existe' });
    }
    
    // Comprobamos si ya existe la relación
    db.query(
      'SELECT * FROM RecompensasObtenidas WHERE id_usuario = ? AND id_recompensa = ?',
      [id_usuario, id_recompensa],
      (err, existingResults) => {
        if (err) return res.status(500).json({ error: err.message });
        
        if (existingResults.length > 0) {
          // Si ya existe, actualizamos
          const query = `
            UPDATE RecompensasObtenidas 
            SET puntos_obtenidos = ?, progreso = ?, completada = ?, fecha_obtencion = NOW()
            WHERE id_usuario = ? AND id_recompensa = ?`;
          
          db.query(
            query,
            [
              puntos_obtenidos || existingResults[0].puntos_obtenidos,
              progreso !== undefined ? progreso : existingResults[0].progreso,
              completada !== undefined ? completada : existingResults[0].completada,
              id_usuario,
              id_recompensa
            ],
            (err, result) => {
              if (err) return res.status(500).json({ error: err.message });
              
              // Si se completó la recompensa, actualizar el progreso del usuario
              if (completada && !existingResults[0].completada) {
                db.beginTransaction(err => {
                  if (err) return res.status(500).json({ error: err.message });
                  
                  // Obtener los puntos de la recompensa
                  db.query(
                    'SELECT puntos_requeridos FROM Recompensas WHERE id_recompensa = ?',
                    [id_recompensa],
                    (err, puntosResults) => {
                      if (err) {
                        return db.rollback(() => {
                          res.status(500).json({ error: err.message });
                        });
                      }
                      
                      const puntosRecompensa = puntosResults[0].puntos_requeridos;
                      
                      // Actualizar el progreso del usuario
                      db.query(
                        `UPDATE ProgresoUsuario 
                         SET puntos = puntos + ?, retos_completados = retos_completados + 1
                         WHERE id_usuario = ?`,
                        [puntos_obtenidos || puntosRecompensa, id_usuario],
                        (err, updateResult) => {
                          if (err) {
                            return db.rollback(() => {
                              res.status(500).json({ error: err.message });
                            });
                          }
                          
                          db.commit(err => {
                            if (err) {
                              return db.rollback(() => {
                                res.status(500).json({ error: err.message });
                              });
                            }
                            
                            res.json({
                              id_usuario,
                              id_recompensa,
                              puntos_obtenidos: puntos_obtenidos || existingResults[0].puntos_obtenidos,
                              progreso: progreso !== undefined ? progreso : existingResults[0].progreso,
                              completada: completada !== undefined ? completada : existingResults[0].completada,
                              fecha_obtencion: new Date()
                            });
                          });
                        }
                      );
                    }
                  );
                });
              } else {
                res.json({
                  id_usuario,
                  id_recompensa,
                  puntos_obtenidos: puntos_obtenidos || existingResults[0].puntos_obtenidos,
                  progreso: progreso !== undefined ? progreso : existingResults[0].progreso,
                  completada: completada !== undefined ? completada : existingResults[0].completada,
                  fecha_obtencion: new Date()
                });
              }
            }
          );
        } else {
          // Si no existe, la creamos
          const query = `
            INSERT INTO RecompensasObtenidas 
            (id_usuario, id_recompensa, puntos_obtenidos, progreso, completada)
            VALUES (?, ?, ?, ?, ?)`;
          
          db.query(
            query,
            [
              id_usuario,
              id_recompensa,
              puntos_obtenidos || 0,
              progreso || 0,
              completada !== undefined ? completada : false
            ],
            (err, result) => {
              if (err) return res.status(500).json({ error: err.message });
              
              // Si se completó la recompensa, actualizar el progreso del usuario
              if (completada) {
                db.beginTransaction(err => {
                  if (err) return res.status(500).json({ error: err.message });
                  
                  // Obtener los puntos de la recompensa
                  db.query(
                    'SELECT puntos_requeridos FROM Recompensas WHERE id_recompensa = ?',
                    [id_recompensa],
                    (err, puntosResults) => {
                      if (err) {
                        return db.rollback(() => {
                          res.status(500).json({ error: err.message });
                        });
                      }
                      
                      const puntosRecompensa = puntosResults[0].puntos_requeridos;
                      
                      // Actualizar el progreso del usuario
                      db.query(
                        `UPDATE ProgresoUsuario 
                         SET puntos = puntos + ?, retos_completados = retos_completados + 1
                         WHERE id_usuario = ?`,
                        [puntos_obtenidos || puntosRecompensa, id_usuario],
                        (err, updateResult) => {
                          if (err) {
                            return db.rollback(() => {
                              res.status(500).json({ error: err.message });
                            });
                          }
                          
                          db.commit(err => {
                            if (err) {
                              return db.rollback(() => {
                                res.status(500).json({ error: err.message });
                              });
                            }
                            
                            res.status(201).json({
                              id_usuario,
                              id_recompensa,
                              puntos_obtenidos: puntos_obtenidos || 0,
                              progreso: progreso || 0,
                              completada: completada !== undefined ? completada : false,
                              fecha_obtencion: new Date()
                            });
                          });
                        }
                      );
                    }
                  );
                });
              } else {
                res.status(201).json({
                  id_usuario,
                  id_recompensa,
                  puntos_obtenidos: puntos_obtenidos || 0,
                  progreso: progreso || 0,
                  completada: completada !== undefined ? completada : false,
                  fecha_obtencion: new Date()
                });
              }
            }
          );
        }
      }
    );
  });
});

// DELETE /api/recompensas/usuario/:idUsuario/recompensa/:idRecompensa - Eliminar una recompensa de un usuario
router.delete('/usuario/:idUsuario/recompensa/:idRecompensa', (req, res) => {
  db.query(
    'DELETE FROM RecompensasObtenidas WHERE id_usuario = ? AND id_recompensa = ?',
    [req.params.idUsuario, req.params.idRecompensa],
    (err, result) => {
      if (err) return res.status(500).json({ error: err.message });
      if (result.affectedRows === 0) return res.status(404).json({ error: 'Relación no encontrada' });
      
      res.json({ message: 'Recompensa eliminada del usuario con éxito' });
    }
  );
});

module.exports = router; 