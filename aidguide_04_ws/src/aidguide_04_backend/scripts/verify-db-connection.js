/**
 * Script para verificar la conexión a la base de datos
 * Utilizado por el workflow de GitHub Actions para validar la configuración
 */

// Función para cargar dotenv de manera segura
function loadEnv() {
  try {
    // Intentar cargar dotenv
    require('dotenv').config();
    console.log('✅ Dotenv cargado correctamente');
  } catch (error) {
    console.warn('⚠️ No se pudo cargar dotenv, usando variables de entorno directamente');
    // En GitHub Actions, las variables ya deberían estar disponibles en process.env
  }
}

// Cargar variables de entorno de manera segura
loadEnv();

// Función para verificar la conexión sin depender de módulos externos
async function verifyDatabaseConnectionDirect() {
  console.log('🔍 Iniciando verificación directa de conexión a la base de datos...');
  console.log(`🔧 Configuración: HOST=${process.env.DB_HOST || 'localhost'}, DB=${process.env.DB_NAME || 'test'}`);

  const mysql = require('mysql2/promise');
  
  try {
    // Crear conexión directamente sin usar el pool
    const connection = await mysql.createConnection({
      host: process.env.DB_HOST || 'localhost',
      user: process.env.DB_USER || 'root',
      password: process.env.DB_PASSWORD || '',
      database: process.env.DB_NAME || 'test'
    });
    
    // Realizar consulta de prueba
    const [rows] = await connection.execute('SELECT 1 as test');
    
    if (rows[0].test === 1) {
      console.log('✅ Conexión a la base de datos establecida correctamente!');
      console.log(`🔐 Conectado a: ${process.env.DB_HOST || 'localhost'} / ${process.env.DB_NAME || 'test'}`);
      await connection.end();
      process.exit(0); // Salida exitosa
    } else {
      console.error('⚠️ La conexión se estableció pero la consulta de prueba falló');
      await connection.end();
      process.exit(1);
    }
  } catch (error) {
    console.error('❌ Error al conectar con la base de datos:');
    console.error(error.message);
    process.exit(1); // Salida con error
  }
}

// Intentar verificar con nuestra implementación robusta primero
verifyDatabaseConnectionDirect(); 