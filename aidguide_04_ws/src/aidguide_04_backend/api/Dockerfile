FROM node:18

WORKDIR /app

# Copia primero los archivos de dependencias
COPY package*.json ./

# Instala las dependencias
RUN npm install
RUN npm install dotenv bcrypt jsonwebtoken uuid --save

# Copia el resto de los archivos
COPY . .

# Expone el puerto 3000
EXPOSE 3000

# Inicia la aplicación
CMD ["node", "app.js"]
