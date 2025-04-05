# Utilizamos una imagen base de Node.js
FROM node:20-alpine AS base

# Instalamos las dependencias necesarias para construcción
FROM base AS deps
WORKDIR /app

# Copiamos los archivos de configuración del proyecto
COPY package.json package-lock.json* ./

# Instalamos dependencias usando npm install en lugar de npm ci
RUN npm install --legacy-peer-deps

# Fase de construcción
FROM base AS builder
WORKDIR /app
COPY --from=deps /app/node_modules ./node_modules
COPY . .

# Construimos la aplicación con flag --legacy-peer-deps
RUN npm run build --legacy-peer-deps

# Fase de producción
FROM base AS runner
WORKDIR /app

ENV NODE_ENV production

# Creamos un usuario no-root para mayor seguridad
RUN addgroup --system --gid 1001 nodejs
RUN adduser --system --uid 1001 nextjs

# Copiamos los archivos necesarios de la etapa de construcción
COPY --from=builder /app/public ./public
COPY --from=builder --chown=nextjs:nodejs /app/.next/standalone ./
COPY --from=builder --chown=nextjs:nodejs /app/.next/static ./.next/static

USER nextjs

EXPOSE 3000

ENV PORT 3000

# Comando para iniciar la aplicación
CMD ["node", "server.js"] 