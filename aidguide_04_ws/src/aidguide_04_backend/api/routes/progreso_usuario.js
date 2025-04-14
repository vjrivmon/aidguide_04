const express = require('express');
const router = express.Router();
const db = require('../db');

// GET /api/progreso-usuario - Obtener progreso de todos los usuarios
router.get('/', (req, res) => {
  db.query('SELECT * FROM ProgresoUsuario', (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/progreso-usuario/:id - Obtener progreso de un usuario por ID
router.get('/:id', (req, res) => {
  db.query('SELECT * FROM ProgresoUsuario WHERE id_usuario = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    
    if (results.length === 0) {
      // Si no existe un registro, devolvemos valores predeterminados
      return res.json({
        id_usuario: parseInt(req.params.id),
        puntos: 0,
        nivel: 1,
        puntos_siguiente_nivel: 350,
        retos_completados: 0
      });
    }
    
    res.json(results[0]);
  });
});

// GET /api/progreso-usuario/ranking/puntos - Obtener ranking de usuarios por puntos
router.get('/ranking/puntos', (req, res) => {
  const query = `
    SELECT p.*, u.nombre, u.apellidos, u.foto_perfil
    FROM ProgresoUsuario p
    JOIN Usuarios u ON p.id_usuario = u.id_usuario
    ORDER BY p.puntos DESC
    LIMIT 10`;
  
  db.query(query, (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/progreso-usuario/ranking/nivel - Obtener ranking de usuarios por nivel
router.get('/ranking/nivel', (req, res) => {
  const query = `
    SELECT p.*, u.nombre, u.apellidos, u.foto_perfil
    FROM ProgresoUsuario p
    JOIN Usuarios u ON p.id_usuario = u.id_usuario
    ORDER BY p.nivel DESC, p.puntos DESC
    LIMIT 10`;
  
  db.query(query, (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/progreso-usuario/ranking/retos - Obtener ranking de usuarios por retos completados
router.get('/ranking/retos', (req, res) => {
  const query = `
    SELECT p.*, u.nombre, u.apellidos, u.foto_perfil
    FROM ProgresoUsuario p
    JOIN Usuarios u ON p.id_usuario = u.id_usuario
    ORDER BY p.retos_completados DESC
    LIMIT 10`;
  
  db.query(query, (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// POST /api/progreso-usuario - Crear o actualizar progreso de un usuario
router.post('/', (req, res) => {
  const { id_usuario, puntos, nivel, puntos_siguiente_nivel, retos_completados } = req.body;
  
  if (!id_usuario) {
    return res.status(400).json({ error: 'El ID de usuario es obligatorio' });
  }
  
  // Verificar si el usuario existe
  db.query('SELECT * FROM Usuarios WHERE id_usuario = ?', [id_usuario], (err, userResults) => {
    if (err) return res.status(500).json({ error: err.message });
    
    if (userResults.length === 0) {
      return res.status(404).json({ error: 'Usuario no encontrado' });
    }
    
    // Verificar si ya existe un registro de progreso para este usuario
    db.query('SELECT * FROM ProgresoUsuario WHERE id_usuario = ?', [id_usuario], (err, progresoResults) => {
      if (err) return res.status(500).json({ error: err.message });
      
      if (progresoResults.length > 0) {
        // Si existe, actualizamos
        const currentProgreso = progresoResults[0];
        
        const updatedProgreso = {
          puntos: puntos !== undefined ? puntos : currentProgreso.puntos,
          nivel: nivel !== undefined ? nivel : currentProgreso.nivel,
          puntos_siguiente_nivel: puntos_siguiente_nivel !== undefined ? puntos_siguiente_nivel : currentProgreso.puntos_siguiente_nivel,
          retos_completados: retos_completados !== undefined ? retos_completados : currentProgreso.retos_completados
        };
        
        const query = `
          UPDATE ProgresoUsuario 
          SET puntos = ?, nivel = ?, puntos_siguiente_nivel = ?, retos_completados = ?
          WHERE id_usuario = ?`;
        
        db.query(
          query,
          [
            updatedProgreso.puntos,
            updatedProgreso.nivel,
            updatedProgreso.puntos_siguiente_nivel,
            updatedProgreso.retos_completados,
            id_usuario
          ],
          (err, result) => {
            if (err) return res.status(500).json({ error: err.message });
            
            // Verificar si el usuario subió de nivel
            if (updatedProgreso.puntos >= currentProgreso.puntos_siguiente_nivel && updatedProgreso.nivel === currentProgreso.nivel) {
              // Subida de nivel
              const newNivel = currentProgreso.nivel + 1;
              const newPuntosSiguienteNivel = Math.round(currentProgreso.puntos_siguiente_nivel * 1.5);
              
              db.query(
                'UPDATE ProgresoUsuario SET nivel = ?, puntos_siguiente_nivel = ? WHERE id_usuario = ?',
                [newNivel, newPuntosSiguienteNivel, id_usuario],
                (err, updateResult) => {
                  if (err) return res.status(500).json({ error: err.message });
                  
                  res.json({
                    id_usuario,
                    puntos: updatedProgreso.puntos,
                    nivel: newNivel,
                    puntos_siguiente_nivel: newPuntosSiguienteNivel,
                    retos_completados: updatedProgreso.retos_completados,
                    nivel_subido: true
                  });
                }
              );
            } else {
              res.json({
                id_usuario,
                ...updatedProgreso,
                nivel_subido: false
              });
            }
          }
        );
      } else {
        // Si no existe, lo creamos
        const defaultProgreso = {
          puntos: puntos || 0,
          nivel: nivel || 1,
          puntos_siguiente_nivel: puntos_siguiente_nivel || 350,
          retos_completados: retos_completados || 0
        };
        
        const query = `
          INSERT INTO ProgresoUsuario (id_usuario, puntos, nivel, puntos_siguiente_nivel, retos_completados)
          VALUES (?, ?, ?, ?, ?)`;
        
        db.query(
          query,
          [
            id_usuario,
            defaultProgreso.puntos,
            defaultProgreso.nivel,
            defaultProgreso.puntos_siguiente_nivel,
            defaultProgreso.retos_completados
          ],
          (err, result) => {
            if (err) return res.status(500).json({ error: err.message });
            
            res.status(201).json({
              id_usuario,
              ...defaultProgreso,
              nivel_subido: false
            });
          }
        );
      }
    });
  });
});

// PUT /api/progreso-usuario/:id/sumar-puntos - Sumar puntos al progreso de un usuario
router.put('/:id/sumar-puntos', (req, res) => {
  const { puntos } = req.body;
  const id_usuario = parseInt(req.params.id);
  
  if (puntos === undefined || isNaN(puntos)) {
    return res.status(400).json({ error: 'La cantidad de puntos es obligatoria' });
  }
  
  db.query('SELECT * FROM ProgresoUsuario WHERE id_usuario = ?', [id_usuario], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    
    if (results.length === 0) {
      // Si no existe un registro, lo creamos
      const defaultProgreso = {
        puntos: puntos,
        nivel: 1,
        puntos_siguiente_nivel: 350,
        retos_completados: 0
      };
      
      const query = `
        INSERT INTO ProgresoUsuario (id_usuario, puntos, nivel, puntos_siguiente_nivel, retos_completados)
        VALUES (?, ?, ?, ?, ?)`;
      
      db.query(
        query,
        [
          id_usuario,
          defaultProgreso.puntos,
          defaultProgreso.nivel,
          defaultProgreso.puntos_siguiente_nivel,
          defaultProgreso.retos_completados
        ],
        (err, result) => {
          if (err) return res.status(500).json({ error: err.message });
          
          res.json({
            id_usuario,
            ...defaultProgreso,
            nivel_subido: false
          });
        }
      );
    } else {
      // Si existe, actualizamos
      const currentProgreso = results[0];
      const newPuntos = currentProgreso.puntos + puntos;
      
      // Verificamos si sube de nivel
      if (newPuntos >= currentProgreso.puntos_siguiente_nivel) {
        const newNivel = currentProgreso.nivel + 1;
        const newPuntosSiguienteNivel = Math.round(currentProgreso.puntos_siguiente_nivel * 1.5);
        
        db.query(
          `UPDATE ProgresoUsuario 
           SET puntos = ?, nivel = ?, puntos_siguiente_nivel = ?
           WHERE id_usuario = ?`,
          [newPuntos, newNivel, newPuntosSiguienteNivel, id_usuario],
          (err, result) => {
            if (err) return res.status(500).json({ error: err.message });
            
            res.json({
              id_usuario,
              puntos: newPuntos,
              nivel: newNivel,
              puntos_siguiente_nivel: newPuntosSiguienteNivel,
              retos_completados: currentProgreso.retos_completados,
              nivel_subido: true,
              puntos_sumados: puntos
            });
          }
        );
      } else {
        // Si no sube de nivel, solo actualizamos los puntos
        db.query(
          'UPDATE ProgresoUsuario SET puntos = ? WHERE id_usuario = ?',
          [newPuntos, id_usuario],
          (err, result) => {
            if (err) return res.status(500).json({ error: err.message });
            
            res.json({
              id_usuario,
              puntos: newPuntos,
              nivel: currentProgreso.nivel,
              puntos_siguiente_nivel: currentProgreso.puntos_siguiente_nivel,
              retos_completados: currentProgreso.retos_completados,
              nivel_subido: false,
              puntos_sumados: puntos
            });
          }
        );
      }
    }
  });
});

// PUT /api/progreso-usuario/:id/subir-nivel - Subir de nivel a un usuario
router.put('/:id/subir-nivel', (req, res) => {
  const id_usuario = parseInt(req.params.id);
  
  db.query('SELECT * FROM ProgresoUsuario WHERE id_usuario = ?', [id_usuario], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    
    if (results.length === 0) {
      // Si no existe un registro, lo creamos en nivel 2
      const defaultProgreso = {
        puntos: 350,
        nivel: 2,
        puntos_siguiente_nivel: 525, // 350 * 1.5
        retos_completados: 0
      };
      
      const query = `
        INSERT INTO ProgresoUsuario (id_usuario, puntos, nivel, puntos_siguiente_nivel, retos_completados)
        VALUES (?, ?, ?, ?, ?)`;
      
      db.query(
        query,
        [
          id_usuario,
          defaultProgreso.puntos,
          defaultProgreso.nivel,
          defaultProgreso.puntos_siguiente_nivel,
          defaultProgreso.retos_completados
        ],
        (err, result) => {
          if (err) return res.status(500).json({ error: err.message });
          
          res.json({
            id_usuario,
            ...defaultProgreso
          });
        }
      );
    } else {
      // Si existe, subimos de nivel
      const currentProgreso = results[0];
      const newNivel = currentProgreso.nivel + 1;
      const newPuntosSiguienteNivel = Math.round(currentProgreso.puntos_siguiente_nivel * 1.5);
      
      db.query(
        `UPDATE ProgresoUsuario 
         SET nivel = ?, puntos_siguiente_nivel = ?
         WHERE id_usuario = ?`,
        [newNivel, newPuntosSiguienteNivel, id_usuario],
        (err, result) => {
          if (err) return res.status(500).json({ error: err.message });
          
          res.json({
            id_usuario,
            puntos: currentProgreso.puntos,
            nivel: newNivel,
            puntos_siguiente_nivel: newPuntosSiguienteNivel,
            retos_completados: currentProgreso.retos_completados
          });
        }
      );
    }
  });
});

// DELETE /api/progreso-usuario/:id - Eliminar el progreso de un usuario
router.delete('/:id', (req, res) => {
  db.query('DELETE FROM ProgresoUsuario WHERE id_usuario = ?', [req.params.id], (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    if (result.affectedRows === 0) return res.status(404).json({ error: 'Progreso no encontrado' });
    
    res.json({ message: 'Progreso eliminado con éxito' });
  });
});

module.exports = router; 