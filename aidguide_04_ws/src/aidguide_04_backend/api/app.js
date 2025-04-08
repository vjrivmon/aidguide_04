const express = require('express');
const swaggerUi = require('swagger-ui-express');
const swaggerDocument = require('./swagger.json');
const usuariosRoutes = require('./routes/usuarios');
const rolesRoutes = require('./routes/roles');
const lugaresRoutes = require('./routes/lugares');
const robotsRoutes = require('./routes/robots');
const rutasRoutes = require('./routes/rutas');
const rutaLugaresRoutes = require('./routes/ruta_lugares');
const recompensasRoutes = require('./routes/recompensas');
const progresoUsuarioRoutes = require('./routes/progreso_usuario');

const app = express();
app.use(express.json());

// Middleware para CORS
app.use((req, res, next) => {
  res.header('Access-Control-Allow-Origin', '*');
  res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept, Authorization');
  res.header('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
  if (req.method === 'OPTIONS') {
    return res.sendStatus(200);
  }
  next();
});

// Rutas de la API
app.use('/api/usuarios', usuariosRoutes);
app.use('/api/roles', rolesRoutes);
app.use('/api/lugares', lugaresRoutes);
app.use('/api/robots', robotsRoutes);
app.use('/api/rutas', rutasRoutes);
app.use('/api/ruta-lugares', rutaLugaresRoutes);
app.use('/api/recompensas', recompensasRoutes);
app.use('/api/progreso-usuario', progresoUsuarioRoutes);

// Documentación Swagger
app.use('/api-docs', swaggerUi.serve, swaggerUi.setup(swaggerDocument));

// Ruta de estado de la API
app.get('/api/status', (req, res) => {
  res.json({ status: 'online', message: 'API AidGuide funcionando correctamente' });
});

// Ruta de health check para Docker
app.get('/health', (req, res) => {
  res.status(200).json({ status: 'ok', timestamp: new Date().toISOString() });
});

// Ruta raíz - redirige a la documentación
app.get('/', (req, res) => {
  res.redirect('/api-docs');
});

const PORT = process.env.PORT || 3000;
app.listen(PORT, () => console.log(`API corriendo en http://localhost:${PORT}/api-docs`));
