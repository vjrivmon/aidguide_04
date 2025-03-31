"use client"

import Link from "next/link"
import dynamic from "next/dynamic"
import { Suspense } from "react"
import { Eye, Shield, Battery, MapPin, MessageSquare, Clock, ArrowRight, Star, CheckCircle } from "lucide-react"

// Importar framer-motion para animaciones
import { motion } from "framer-motion"

// Importar el componente de modelo 3D de forma dinámica para evitar errores de SSR
const RobotModel = dynamic(() => import("@/components/robot-model"), {
  ssr: false,
  loading: () => (
    <div className="w-full h-80 md:h-96 flex items-center justify-center bg-gradient-to-b from-blue-50 to-gray-100 rounded-33 shadow-lg">
      <div className="animate-pulse flex flex-col items-center">
        <div className="w-24 h-24 bg-blue-200 rounded-full mb-4"></div>
        <div className="h-4 w-32 bg-blue-200 rounded mb-2"></div>
        <div className="h-3 w-24 bg-blue-100 rounded"></div>
      </div>
    </div>
  ),
})

export default function Home() {
  return (
    <div className="flex flex-col">
      {/* Hero Section - Mejorado con gradientes y efectos visuales */}
      <section className="bg-gradient-to-br from-background via-background to-blue-100 py-16 md:py-24 relative overflow-hidden">
        {/* Elementos decorativos de fondo */}
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute -top-40 -right-40 w-80 h-80 bg-blue-200 rounded-full opacity-20 blur-3xl"></div>
          <div className="absolute top-60 -left-20 w-60 h-60 bg-blue-300 rounded-full opacity-10 blur-3xl"></div>
        </div>

        <div className="container-custom relative z-10">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-12 items-center">
            <motion.div initial={{ opacity: 0, y: 20 }} animate={{ opacity: 1, y: 0 }} transition={{ duration: 0.8 }}>
              <div className="inline-block px-3 py-1 bg-green-300 text-black rounded-33 text-sm font-medium mb-6">
                Tecnología Inclusiva
              </div>
              <h1 className="text-3xl md:text-4xl lg:text-5xl font-bold mb-6 leading-tight">
                AidGuide: Navega el mundo con confianza y seguridad
              </h1>
              <p className="text-lg mb-8 text-gray-700">
                Perro guía para personas invidentes y con movilidad reducida con inteligencia artificial y visión en tiempo real para crear un entorno
                más seguro e inclusivo.
              </p>
              <div className="flex flex-col sm:flex-row gap-4">
                <Link
                  href="/register"
                  className="btn-primary text-center group transition-all duration-300 transform hover:translate-y-[-2px] hover:shadow-lg"
                >
                  <span className="flex items-center justify-center">
                    Comenzar ahora
                    <ArrowRight className="ml-2 h-4 w-4 transition-transform group-hover:translate-x-1" />
                  </span>
                </Link>
                <Link
                  href="/product"
                  className="btn-secondary text-center transition-all duration-300 transform hover:translate-y-[-2px] hover:shadow-md"
                >
                  Conocer más
                </Link>
              </div>

              {/* Indicadores de confianza */}
              <div className="mt-8 flex items-center text-sm text-gray-600">
                <div className="flex items-center mr-6">
                  <CheckCircle className="h-4 w-4 text-green-500 mr-1" />
                  <span>Tecnología certificada</span>
                </div>
                <div className="flex items-center">
                  <div className="flex">
                    {[...Array(5)].map((_, i) => (
                      <Star key={i} className="h-4 w-4 text-yellow-400" fill="#FACC15" />
                    ))}
                  </div>
                  <span className="ml-1">4.9/5 (120+ usuarios)</span>
                </div>
              </div>
            </motion.div>

            <motion.div
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ duration: 0.8, delay: 0.3 }}
              className="relative"
            >
              <Suspense
                fallback={
                  <div className="w-full h-80 md:h-96 flex items-center justify-center bg-gradient-to-b from-blue-50 to-gray-100 rounded-33 shadow-lg">
                    <div className="animate-pulse flex flex-col items-center">
                      <div className="w-24 h-24 bg-blue-200 rounded-full mb-4"></div>
                      <div className="h-4 w-32 bg-blue-200 rounded mb-2"></div>
                      <div className="h-3 w-24 bg-blue-100 rounded"></div>
                    </div>
                  </div>
                }
              >
                <RobotModel />
              </Suspense>
            </motion.div>
          </div>
        </div>
      </section>

      {/* Features Section - Mejorado con tarjetas más atractivas */}
      <section className="py-16 bg-white">
        <div className="container-custom">
          <motion.div
            className="text-center mb-12"
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.8 }}
          >
            <h2 className="text-2xl md:text-3xl font-bold mb-4 relative inline-block">
              Características Principales
              <div className="absolute bottom-0 left-1/2 transform -translate-x-1/2 w-24 h-1 bg-button rounded-full"></div>
            </h2>
            <p className="max-w-2xl mx-auto text-gray-600 mt-6">
              Nuestro robot guía está diseñado para proporcionar la máxima autonomía y seguridad a sus usuarios.
            </p>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8">
            {[
              {
                icon: <Eye size={24} />,
                title: "Detección de Obstáculos",
                description:
                  "Identifica obstáculos, señales de tránsito y semáforos para garantizar un desplazamiento seguro.",
              },
              {
                icon: <Shield size={24} />,
                title: "Alertas de Seguridad",
                description:
                  "Notificaciones sobre proximidad de escaleras, cambios de nivel en el suelo y otros peligros potenciales.",
              },
              {
                icon: <Battery size={24} />,
                title: "Autonomía Extendida",
                description:
                  "Batería de larga duración que garantiza operatividad durante todo el día sin necesidad de recargas constantes.",
              },
              {
                icon: <MapPin size={24} />,
                title: "Orientación Precisa",
                description:
                  "Guía hacia paradas de transporte público y puntos de interés para facilitar los desplazamientos diarios.",
              },
              {
                icon: <MessageSquare size={24} />,
                title: "Control por Voz",
                description: "Interacción mediante comandos de voz, eliminando la dependencia de interfaces visuales.",
              },
              {
                icon: <Clock size={24} />,
                title: "Tiempo Real",
                description:
                  "Procesamiento de información en tiempo real para una respuesta inmediata ante cualquier situación.",
              },
            ].map((feature, index) => (
              <motion.div
                key={index}
                className="card hover:shadow-lg transition-all duration-300 transform hover:-translate-y-1 border border-gray-100"
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
              >
                <div className="flex items-center mb-4">
                  <div className="bg-button p-3 rounded-full text-white mr-4 shadow-md">{feature.icon}</div>
                  <h3 className="font-semibold">{feature.title}</h3>
                </div>
                <p className="text-gray-600">{feature.description}</p>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Testimonials Section - Mejorado con diseño más moderno */}
      <section className="py-16 bg-gradient-to-br from-background to-blue-50">
        <div className="container-custom">
          <motion.div
            className="text-center mb-12"
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.8 }}
          >
            <h2 className="text-2xl md:text-3xl font-bold mb-4 relative inline-block">
              Testimonios
              <div className="absolute bottom-0 left-1/2 transform -translate-x-1/2 w-16 h-1 bg-button rounded-full"></div>
            </h2>
            <p className="max-w-2xl mx-auto text-gray-600 mt-6">
              Descubre cómo AidGuide está transformando la vida de sus usuarios.
            </p>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            {[
              {
                quote:
                  "AidGuide ha cambiado completamente mi forma de desplazarme por la ciudad. Me siento mucho más seguro y autónomo.",
                name: "Carlos Martínez",
                role: "Usuario desde 2023",
              },
              {
                quote:
                  "La precisión con la que detecta obstáculos y me alerta de peligros es impresionante. Ahora puedo ir a lugares que antes evitaba.",
                name: "Laura Sánchez",
                role: "Usuaria desde 2022",
              },
              {
                quote:
                  "La autonomía de la batería es excelente. Puedo usarlo todo el día sin preocuparme por quedarse sin energía en medio de un trayecto.",
                name: "Miguel Fernández",
                role: "Usuario desde 2023",
              },
            ].map((testimonial, index) => (
              <motion.div
                key={index}
                className="card border border-gray-100 hover:shadow-lg transition-all duration-300"
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
              >
                <div className="flex items-center mb-4">
                  <div className="flex">
                    {[...Array(5)].map((_, i) => (
                      <Star key={i} className="h-4 w-4 text-yellow-400" fill="#FACC15" />
                    ))}
                  </div>
                </div>
                <p className="italic mb-6 text-gray-700">"{testimonial.quote}"</p>
                <div className="flex items-center mt-auto">
                  <div className="w-12 h-12 bg-gradient-to-br from-blue-400 to-button rounded-full mr-4 flex items-center justify-center text-white font-bold">
                    {testimonial.name.charAt(0)}
                  </div>
                  <div>
                    <h4 className="font-semibold">{testimonial.name}</h4>
                    <p className="text-sm text-gray-600">{testimonial.role}</p>
                  </div>
                </div>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* CTA Section - Mejorado con diseño más atractivo */}
      <section className="py-16 bg-gradient-to-r from-button to-blue-700 text-white relative overflow-hidden">
        {/* Elementos decorativos */}
        <div className="absolute inset-0 overflow-hidden">
          <div className="absolute top-0 left-0 w-full h-20 bg-white opacity-5 transform -skew-y-6"></div>
          <div className="absolute bottom-0 right-0 w-full h-20 bg-white opacity-5 transform skew-y-6"></div>
        </div>

        <div className="container-custom text-center relative z-10">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.8 }}
          >
            <h2 className="text-2xl md:text-3xl font-bold mb-6">
              ¿Listo para experimentar una nueva forma de movilidad?
            </h2>
            <p className="max-w-2xl mx-auto mb-8 text-lg opacity-90">
              Únete a nuestra comunidad y descubre cómo AidGuide puede transformar tu vida cotidiana.
            </p>
            <div className="flex flex-col sm:flex-row justify-center gap-4">
              <Link
                href="/register"
                className="bg-white text-button py-3 px-8 rounded-33 hover:bg-gray-100 transition-all duration-300 font-medium shadow-lg hover:shadow-xl transform hover:-translate-y-1"
              >
                Registrarse ahora
              </Link>
              <Link
                href="/contact"
                className="border border-white py-3 px-8 rounded-33 hover:bg-white/10 transition-all duration-300 font-medium"
              >
                Contactar con nosotros
              </Link>
            </div>

            {/* Añadir un indicador de confianza */}
            <div className="mt-10 flex justify-center">
              <div className="bg-white/10 rounded-33 px-6 py-3 inline-flex items-center">
                <Shield className="h-5 w-5 mr-2 text-white" />
                <span>Tecnología segura y certificada por expertos en accesibilidad</span>
              </div>
            </div>
          </motion.div>
        </div>
      </section>
    </div>
  )
}

