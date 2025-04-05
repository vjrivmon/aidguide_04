const express = require('express');
const router = express.Router();
const db = require('../db');

// GET /api/roles - Obtener todos los roles
router.get('/', (req, res) => {
  db.query('SELECT * FROM Roles', (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/roles/:id - Obtener un rol por ID
router.get('/:id', (req, res) => {
  db.query('SELECT * FROM Roles WHERE id_rol = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Rol no encontrado' });
    res.json(results[0]);
  });
});

// POST /api/roles - Crear un nuevo rol
router.post('/', (req, res) => {
  const { nombre } = req.body;
  if (!nombre) return res.status(400).json({ error: 'El nombre del rol es obligatorio' });
  
  db.query('INSERT INTO Roles (nombre) VALUES (?)', [nombre], (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    res.status(201).json({ id_rol: result.insertId, nombre });
  });
});

// PUT /api/roles/:id - Actualizar un rol
router.put('/:id', (req, res) => {
  const { nombre } = req.body;
  if (!nombre) return res.status(400).json({ error: 'El nombre del rol es obligatorio' });
  
  db.query('UPDATE Roles SET nombre = ? WHERE id_rol = ?', [nombre, req.params.id], (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    if (result.affectedRows === 0) return res.status(404).json({ error: 'Rol no encontrado' });
    res.json({ id_rol: parseInt(req.params.id), nombre });
  });
});

// DELETE /api/roles/:id - Eliminar un rol
router.delete('/:id', (req, res) => {
  db.query('DELETE FROM Roles WHERE id_rol = ?', [req.params.id], (err, result) => {
    if (err) return res.status(500).json({ error: err.message });
    if (result.affectedRows === 0) return res.status(404).json({ error: 'Rol no encontrado' });
    res.json({ message: 'Rol eliminado con éxito' });
  });
});

module.exports = router; 