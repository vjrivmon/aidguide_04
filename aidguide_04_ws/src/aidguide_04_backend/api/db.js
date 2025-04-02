const mysql = require('mysql2');
require('dotenv').config();

/**
 * Configuración del pool de conexiones a la base de datos MySQL
 * Utilizando variables de entorno para gestionar la información sensible
 */
const pool = mysql.createPool({
  host: process.env.DB_HOST || 'mysql',
  user: process.env.DB_USER || 'aiduser',
  password: process.env.DB_PASSWORD || 'password123',
  database: process.env.DB_NAME || 'AidGuide',
});

module.exports = pool;
