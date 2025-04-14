"use client"

import { useState } from "react"
import { ChevronDown, ChevronUp } from "lucide-react"

export default function FAQ() {
  const faqs = [
    {
      category: "Producto",
      questions: [
        {
          question: "¿Qué es AidGuide?",
          answer:
            "AidGuide es un robot guía diseñado específicamente para personas invidentes. Utiliza tecnologías avanzadas como inteligencia artificial, visión por computadora y sensores para ayudar a las personas con discapacidad visual a desplazarse de forma segura y autónoma en entornos urbanos.",
        },
        {
          question: "¿Cómo funciona AidGuide?",
          answer:
            "AidGuide utiliza múltiples sensores (cámaras, LiDAR, ultrasonido) para percibir el entorno. La información es procesada por algoritmos de inteligencia artificial que identifican obstáculos, señales y rutas seguras. El robot guía al usuario mediante comandos de voz y movimiento físico, proporcionando una experiencia de navegación segura y eficiente.",
        },
        {
          question: "¿Cuánto tiempo dura la batería?",
          answer:
            "La batería de AidGuide tiene una autonomía de hasta 12 horas de uso continuo, lo que garantiza que pueda funcionar durante toda la jornada sin necesidad de recargas. El tiempo de carga completa es de aproximadamente 3 horas.",
        },
        {
          question: "¿Es resistente a la lluvia?",
          answer:
            "AidGuide tiene una certificación IPX4, lo que significa que es resistente a salpicaduras de agua desde cualquier dirección. Puede utilizarse bajo lluvia ligera, pero no se recomienda su uso en condiciones de lluvia intensa o sumergirlo en agua.",
        },
      ],
    },
    {
      category: "Uso y Funcionalidades",
      questions: [
        {
          question: "¿Cómo interactúo con el robot?",
          answer:
            "La interacción principal con AidGuide se realiza mediante comandos de voz. El robot cuenta con reconocimiento de voz avanzado que permite una comunicación natural y conversacional. También dispone de una aplicación móvil accesible para configuración y funciones avanzadas.",
        },
        {
          question: "¿Puede detectar semáforos y señales de tráfico?",
          answer:
            "Sí, AidGuide está equipado con tecnología de visión por computadora que le permite reconocer e interpretar semáforos, señales de tráfico, pasos de peatones y otras indicaciones relevantes para la seguridad vial.",
        },
        {
          question: "¿Funciona en interiores y exteriores?",
          answer:
            "Sí, AidGuide está diseñado para funcionar tanto en entornos interiores (edificios, centros comerciales, oficinas) como en exteriores (calles, parques, plazas). Se adapta automáticamente a las características de cada entorno.",
        },
        {
          question: "¿Puede guiarme a destinos específicos?",
          answer:
            "Sí, AidGuide puede guiarte a destinos específicos utilizando una combinación de GPS, mapas digitales y reconocimiento del entorno. Puedes indicar el destino mediante comandos de voz o seleccionarlo desde la aplicación móvil.",
        },
      ],
    },
    {
      category: "Adquisición y Soporte",
      questions: [
        {
          question: "¿Cómo puedo adquirir un AidGuide?",
          answer:
            "AidGuide está disponible en diferentes modalidades: compra, alquiler o suscripción mensual. Puedes solicitar información detallada a través de nuestro formulario de contacto o llamando a nuestro servicio de atención al cliente.",
        },
        {
          question: "¿Ofrecen demostraciones del producto?",
          answer:
            "Sí, ofrecemos demostraciones gratuitas para que puedas experimentar las capacidades de AidGuide antes de tomar una decisión. Puedes solicitar una demostración a través de nuestra web o contactando con nuestro equipo de ventas.",
        },
        {
          question: "¿Qué incluye la garantía?",
          answer:
            "Todos nuestros productos incluyen una garantía de 2 años que cubre defectos de fabricación y funcionamiento. Además, ofrecemos planes de garantía extendida y servicios de mantenimiento preventivo para asegurar el óptimo funcionamiento del robot.",
        },
        {
          question: "¿Cómo es el servicio de soporte técnico?",
          answer:
            "Disponemos de un servicio de soporte técnico especializado disponible por teléfono, email y chat. Dependiendo del plan contratado, el soporte puede ser en horario laboral o 24/7. También ofrecemos asistencia remota para resolver problemas de software.",
        },
      ],
    },
    {
      category: "Tecnología y Seguridad",
      questions: [
        {
          question: "¿Qué tecnologías utiliza AidGuide?",
          answer:
            "AidGuide integra múltiples tecnologías avanzadas: inteligencia artificial, aprendizaje automático, visión por computadora, procesamiento de lenguaje natural, sensores LiDAR, cámaras estéreo, ultrasonido y conectividad 4G/5G, entre otras.",
        },
        {
          question: "¿Es seguro para el uso diario?",
          answer:
            "Sí, AidGuide ha sido diseñado priorizando la seguridad del usuario. Cuenta con múltiples sistemas redundantes, detección de fallos, alertas preventivas y mecanismos de parada de emergencia. Además, cumple con todas las normativas de seguridad aplicables.",
        },
        {
          question: "¿Cómo protege mi privacidad?",
          answer:
            "La privacidad es una prioridad para nosotros. Toda la información capturada por los sensores se procesa localmente en el robot siempre que es posible. Los datos que se envían a la nube están encriptados y se utilizan únicamente para mejorar el servicio, siguiendo estrictas políticas de privacidad y cumpliendo con el RGPD.",
        },
        {
          question: "¿Se actualiza el software automáticamente?",
          answer:
            "Sí, AidGuide recibe actualizaciones de software automáticas que incluyen mejoras de funcionalidad, correcciones de errores y actualizaciones de seguridad. Las actualizaciones se realizan durante periodos de inactividad para no interrumpir el uso normal.",
        },
      ],
    },
  ]

  const [openCategory, setOpenCategory] = useState<string | null>("Producto")
  const [openQuestions, setOpenQuestions] = useState<Record<string, boolean>>({})

  const toggleCategory = (category: string) => {
    setOpenCategory(openCategory === category ? null : category)
  }

  const toggleQuestion = (category: string, index: number) => {
    const key = `${category}-${index}`
    setOpenQuestions((prev) => ({
      ...prev,
      [key]: !prev[key],
    }))
  }

  return (
    <div className="bg-background min-h-screen py-16">
      <div className="container-custom">
        <div className="text-center mb-12">
          <h1 className="text-3xl md:text-4xl font-bold mb-4">Preguntas Frecuentes</h1>
          <p className="max-w-3xl mx-auto text-lg">Encuentra respuestas a las preguntas más comunes sobre AidGuide.</p>
        </div>

        <div className="max-w-3xl mx-auto">
          {faqs.map((category) => (
            <div key={category.category} className="mb-6">
              <button
                className={`w-full flex justify-between items-center p-4 rounded-33 ${
                  openCategory === category.category ? "bg-button text-white" : "bg-white hover:bg-gray-50"
                } transition-colors duration-200 shadow-md`}
                onClick={() => toggleCategory(category.category)}
                aria-expanded={openCategory === category.category}
              >
                <h2 className="text-xl font-bold">{category.category}</h2>
                {openCategory === category.category ? <ChevronUp size={24} /> : <ChevronDown size={24} />}
              </button>

              {openCategory === category.category && (
                <div className="mt-4 space-y-4">
                  {category.questions.map((faq, index) => (
                    <div key={index} className="bg-white rounded-33 overflow-hidden shadow-sm border border-gray-100">
                      <button
                        className="w-full flex justify-between items-center p-4 text-left hover:bg-gray-50 transition-colors duration-200"
                        onClick={() => toggleQuestion(category.category, index)}
                        aria-expanded={openQuestions[`${category.category}-${index}`]}
                      >
                        <h3 className="text-lg font-medium">{faq.question}</h3>
                        {openQuestions[`${category.category}-${index}`] ? (
                          <ChevronUp size={20} className="text-button flex-shrink-0" />
                        ) : (
                          <ChevronDown size={20} className="text-button flex-shrink-0" />
                        )}
                      </button>

                      {openQuestions[`${category.category}-${index}`] && (
                        <div className="p-4 pt-0 border-t">
                          <p className="text-gray-600">{faq.answer}</p>
                        </div>
                      )}
                    </div>
                  ))}
                </div>
              )}
            </div>
          ))}
        </div>

        <div className="mt-12 text-center">
          <h3 className="text-xl font-bold mb-4">¿No encuentras lo que buscas?</h3>
          <p className="mb-6">Si tienes alguna pregunta adicional, no dudes en contactarnos.</p>
          <div className="flex flex-col sm:flex-row justify-center gap-4">
            <a href="/contact" className="btn-primary">
              Contactar con soporte
            </a>
            <a href="tel:+34123456789" className="btn-secondary">
              Llamar: +34 123 456 789
            </a>
          </div>
        </div>
      </div>
    </div>
  )
}

