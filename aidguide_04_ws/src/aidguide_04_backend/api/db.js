const mysql = require('mysql2');
const path = require('path');
const fs = require('fs');

// Intentar cargar dotenv desde múltiples ubicaciones para mayor robustez
const envPaths = [
  path.resolve(__dirname, '../.env'),
  path.resolve(process.cwd(), '.env')
];

let envLoaded = false;
for (const envPath of envPaths) {
  if (fs.existsSync(envPath)) {
    require('dotenv').config({ path: envPath });
    console.log(`Variables de entorno cargadas desde: ${envPath}`);
    envLoaded = true;
    break;
  }
}

if (!envLoaded) {
  console.warn('No se encontró archivo .env. Usando valores por defecto.');
  require('dotenv').config();
}

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
