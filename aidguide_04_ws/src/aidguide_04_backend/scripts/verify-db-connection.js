/**
 * Script para verificar la conexión a la base de datos
 * Utilizado por el workflow de GitHub Actions para validar la configuración
 */
require('dotenv').config(); // Cargar variables de entorno primero
const pool = require('../api/db');

async function verifyDatabaseConnection() {
  console.log('🔍 Iniciando verificación de conexión a la base de datos...');
  console.log(`🔧 Configuración: HOST=${process.env.DB_HOST}, DB=${process.env.DB_NAME}, USER=${process.env.DB_USER}`);
  
  try {
    // Realizamos una consulta simple para verificar la conexión
    const [result] = await pool.promise().query('SELECT 1 as test');
    
    if (result[0].test === 1) {
      console.log('✅ Conexión a la base de datos establecida correctamente!');
      console.log(`🔐 Conectado a: ${process.env.DB_HOST} / ${process.env.DB_NAME}`);
      process.exit(0); // Salida exitosa
    } else {
      console.error('⚠️ La conexión se estableció pero la consulta de prueba falló');
      process.exit(1);
    }
  } catch (error) {
    console.error('❌ Error al conectar con la base de datos:');
    console.error(error.message);
    process.exit(1); // Salida con error
  }
}

// Ejecutar la verificación
verifyDatabaseConnection(); 