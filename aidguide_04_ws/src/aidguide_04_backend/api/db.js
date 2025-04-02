const mysql = require('mysql2');

const pool = mysql.createPool({
  host: 'mysql',         // nombre del servicio en docker-compose
  user: 'aiduser',
  password: 'password123',
  database: 'AidGuide',
});

module.exports = pool;
