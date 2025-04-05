const express = require('express');
const router = express.Router();
const db = require('../db');

// GET /api/usuarios - Obtener todos los usuarios
router.get('/', (req, res) => {
  db.query('SELECT * FROM Usuarios', (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    res.json(results);
  });
});

// GET /api/usuarios/:id - Obtener un usuario por ID
router.get('/:id', (req, res) => {
  db.query('SELECT * FROM Usuarios WHERE id_usuario = ?', [req.params.id], (err, results) => {
    if (err) return res.status(500).json({ error: err.message });
    if (results.length === 0) return res.status(404).json({ error: 'Usuario no encontrado' });
    res.json(results[0]);
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

module.exports = router;
