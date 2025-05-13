# Workflows de GitHub Actions

## Validación de Conexión a Base de Datos

Este workflow se encarga de verificar que la conexión a la base de datos funciona correctamente cuando se realiza un push a la rama `develop`.

### Funcionalidades

- Se ejecuta automáticamente al hacer push a la rama `develop` o al crear un pull request hacia dicha rama
- Configura un entorno MySQL para pruebas
- Utiliza los secrets de GitHub para configurar las credenciales
- Ejecuta un script de verificación que asegura que la conexión funciona correctamente

### Secrets requeridos

Para que este workflow funcione correctamente, necesitas configurar los siguientes secrets en tu repositorio:

- `DB_HOST`: Host de la base de datos (en el workflow se usa localhost)
- `DB_USER`: Usuario de la base de datos
- `DB_PASSWORD`: Contraseña del usuario
- `DB_NAME`: Nombre de la base de datos

### Cómo configurar los secrets

1. Ve a tu repositorio en GitHub
2. Navega a "Settings" > "Secrets and variables" > "Actions"
3. Haz clic en "New repository secret"
4. Añade cada uno de los secrets mencionados con sus valores correspondientes

### Diagrama de flujo del workflow

```
Commit/PR a develop → Iniciar workflow → Configurar MySQL → Instalar dependencias → Verificar conexión → ✅/❌
``` 