import Image from "next/image"
import Link from "next/link"
import { Check } from "lucide-react"

export default function Product() {
  const features = [
    {
      title: "Detección avanzada de obstáculos",
      description:
        "Identifica y evita obstáculos en tiempo real, incluyendo objetos a nivel del suelo, a la altura de la cabeza y en movimiento.",
    },
    {
      title: "Reconocimiento de señales y semáforos",
      description: "Interpreta señales de tránsito y estados de semáforos para garantizar cruces seguros.",
    },
    {
      title: "Alertas de proximidad",
      description: "Notifica sobre la cercanía de escaleras, cambios de nivel, obras y otros peligros potenciales.",
    },
    {
      title: "Batería de larga duración",
      description: "Autonomía de hasta 12 horas de uso continuo, suficiente para toda la jornada.",
    },
    {
      title: "Navegación a puntos de interés",
      description:
        "Guía hacia paradas de transporte público, comercios, edificios públicos y otros destinos frecuentes.",
    },
    {
      title: "Control por voz",
      description: "Interfaz completamente controlable mediante comandos de voz naturales y conversacionales.",
    },
    {
      title: "Detección de personas",
      description: "Identifica la presencia de personas en el entorno y ayuda a evitar colisiones.",
    },
    {
      title: "Resistente a condiciones climáticas",
      description:
        "Diseñado para funcionar en diversas condiciones ambientales, incluyendo lluvia ligera y temperaturas extremas.",
    },
  ]

  const specifications = [
    { name: "Dimensiones", value: "40 x 30 x 90 cm" },
    { name: "Peso", value: "8 kg" },
    { name: "Autonomía", value: "Hasta 12 horas" },
    { name: "Tiempo de carga", value: "3 horas" },
    { name: "Velocidad máxima", value: "5 km/h" },
    { name: "Sensores", value: "Cámaras estéreo, LiDAR, ultrasonido, infrarrojos" },
    { name: "Conectividad", value: "Wi-Fi, Bluetooth, 4G/5G" },
    { name: "Resistencia al agua", value: "IPX4 (resistente a salpicaduras)" },
    { name: "Temperatura de operación", value: "-10°C a 40°C" },
  ]

  return (
    <div className="bg-background min-h-screen">
      {/* Hero Section */}
      <section className="py-16 md:py-24 bg-button text-white">
        <div className="container-custom">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-12 items-center">
            <div>
              <h1 className="text-3xl md:text-4xl lg:text-5xl font-bold mb-6 leading-tight">AidGuide Robot</h1>
              <p className="text-lg mb-8">El compañero perfecto para una movilidad segura, autónoma e inclusiva.</p>
              <div className="flex flex-col sm:flex-row gap-4">
                <Link
                  href="/register"
                  className="bg-white text-button py-2 px-6 rounded-33 hover:bg-gray-100 transition-colors font-medium shadow-md"
                >
                  Solicitar demostración
                </Link>
                <Link
                  href="/contact"
                  className="border border-white py-2 px-6 rounded-33 hover:bg-white/10 transition-colors font-medium"
                >
                  Contactar con ventas
                </Link>
              </div>
            </div>
            <div className="flex justify-center">
              <div className="relative w-full max-w-md h-80 md:h-96">
                <Image src="/logo.svg" alt="AidGuide Robot" fill className="object-contain" />
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className="py-16">
        <div className="container-custom">
          <div className="text-center mb-12">
            <h2 className="text-2xl md:text-3xl font-bold mb-4">Características Principales</h2>
            <p className="max-w-3xl mx-auto">
              Descubre todas las funcionalidades que hacen de AidGuide la solución más completa para la movilidad de
              personas invidentes.
            </p>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-8">
            {features.map((feature, index) => (
              <div key={index} className="card hover:shadow-lg transition-all duration-300">
                <h3 className="text-lg font-bold mb-2 text-button">{feature.title}</h3>
                <p className="text-gray-600">{feature.description}</p>
              </div>
            ))}
          </div>
        </div>
      </section>

      {/* How It Works */}
      <section className="py-16 bg-white">
        <div className="container-custom">
          <div className="text-center mb-12">
            <h2 className="text-2xl md:text-3xl font-bold mb-4">Cómo Funciona</h2>
            <p className="max-w-3xl mx-auto">
              AidGuide utiliza tecnología avanzada para proporcionar una experiencia de guía segura y eficiente.
            </p>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            <div className="card text-center hover:shadow-lg transition-all duration-300">
              <div className="bg-button text-white w-12 h-12 rounded-full flex items-center justify-center mx-auto mb-4">
                1
              </div>
              <h3 className="text-xl font-bold mb-2">Percepción</h3>
              <p>
                Múltiples sensores capturan información del entorno en tiempo real, creando un mapa tridimensional del
                espacio.
              </p>
            </div>
            <div className="card text-center hover:shadow-lg transition-all duration-300">
              <div className="bg-button text-white w-12 h-12 rounded-full flex items-center justify-center mx-auto mb-4">
                2
              </div>
              <h3 className="text-xl font-bold mb-2">Procesamiento</h3>
              <p>
                La inteligencia artificial analiza los datos, identifica obstáculos, señales y determina la ruta más
                segura.
              </p>
            </div>
            <div className="card text-center hover:shadow-lg transition-all duration-300">
              <div className="bg-button text-white w-12 h-12 rounded-full flex items-center justify-center mx-auto mb-4">
                3
              </div>
              <h3 className="text-xl font-bold mb-2">Guía</h3>
              <p>
                El robot proporciona indicaciones precisas mediante comandos de voz y movimiento físico, guiando al
                usuario de forma segura.
              </p>
            </div>
          </div>
        </div>
      </section>

      {/* Technical Specifications */}
      <section className="py-16">
        <div className="container-custom">
          <div className="text-center mb-12">
            <h2 className="text-2xl md:text-3xl font-bold mb-4">Especificaciones Técnicas</h2>
            <p className="max-w-3xl mx-auto">Conoce en detalle las características técnicas de AidGuide.</p>
          </div>

          <div className="card hover:shadow-lg transition-all duration-300">
            <div className="grid grid-cols-1 md:grid-cols-2 gap-8">
              <div>
                <h3 className="text-xl font-bold mb-4 text-button">Especificaciones</h3>
                <ul className="space-y-3">
                  {specifications.map((spec, index) => (
                    <li key={index} className="flex">
                      <span className="font-medium w-48">{spec.name}:</span>
                      <span>{spec.value}</span>
                    </li>
                  ))}
                </ul>
              </div>
              <div>
                <h3 className="text-xl font-bold mb-4 text-button">Contenido del Paquete</h3>
                <ul className="space-y-3">
                  <li className="flex items-start">
                    <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                    <span>Robot AidGuide</span>
                  </li>
                  <li className="flex items-start">
                    <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                    <span>Base de carga</span>
                  </li>
                  <li className="flex items-start">
                    <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                    <span>Cable de alimentación</span>
                  </li>
                  <li className="flex items-start">
                    <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                    <span>Guía de inicio rápido en braille y audio</span>
                  </li>
                  <li className="flex items-start">
                    <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                    <span>Manual de usuario completo (digital)</span>
                  </li>
                  <li className="flex items-start">
                    <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                    <span>Tarjeta SIM con 1 año de conectividad incluida</span>
                  </li>
                  <li className="flex items-start">
                    <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                    <span>Garantía de 2 años</span>
                  </li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* Pricing */}
      <section className="py-16 bg-white">
        <div className="container-custom">
          <div className="text-center mb-12">
            <h2 className="text-2xl md:text-3xl font-bold mb-4">Planes y Precios</h2>
            <p className="max-w-3xl mx-auto">Ofrecemos diferentes opciones para adaptarnos a tus necesidades.</p>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            <div className="card border border-gray-200 hover:border-button transition-colors">
              <div className="text-center mb-6">
                <h3 className="text-xl font-bold">Plan Básico</h3>
                <p className="text-gray-600 mt-2">Para uso personal ocasional</p>
                <div className="mt-4">
                  <span className="text-3xl font-bold">€99</span>
                  <span className="text-gray-600">/mes</span>
                </div>
              </div>
              <ul className="space-y-3 mb-8">
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Robot AidGuide (en alquiler)</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Conectividad básica</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Soporte técnico en horario laboral</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Actualizaciones de software</span>
                </li>
              </ul>
              <div className="text-center">
                <button className="btn-secondary w-full block cursor-pointer">
                  Solicitar información
                </button>
              </div>
            </div>

            <div className="card border-2 border-button relative transform scale-105 shadow-lg">
              <div className="absolute top-0 right-0 bg-button text-white px-4 py-1 text-sm font-medium rounded-bl-lg rounded-tr-33">
                Recomendado
              </div>
              <div className="text-center mb-6">
                <h3 className="text-xl font-bold">Plan Premium</h3>
                <p className="text-gray-600 mt-2">Para uso personal diario</p>
                <div className="mt-4">
                  <span className="text-3xl font-bold">€149</span>
                  <span className="text-gray-600">/mes</span>
                </div>
              </div>
              <ul className="space-y-3 mb-8">
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Robot AidGuide (en alquiler)</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Conectividad avanzada 4G/5G</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Soporte técnico 24/7</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Actualizaciones de software prioritarias</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Mantenimiento preventivo trimestral</span>
                </li>
              </ul>
              <div className="text-center">
                <button className="btn-primary w-full block cursor-pointer">
                  Solicitar información
                </button>
              </div>
            </div>

            <div className="card border border-gray-200 hover:border-button transition-colors">
              <div className="text-center mb-6">
                <h3 className="text-xl font-bold">Plan Empresarial</h3>
                <p className="text-gray-600 mt-2">Para organizaciones</p>
                <div className="mt-4">
                  <span className="text-3xl font-bold">Consultar</span>
                </div>
              </div>
              <ul className="space-y-3 mb-8">
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Múltiples robots AidGuide</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Conectividad premium dedicada</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Soporte técnico VIP</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Personalización según necesidades</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Formación para personal</span>
                </li>
                <li className="flex items-start">
                  <Check className="text-green-500 mr-2 mt-1 flex-shrink-0" size={18} />
                  <span>Integración con sistemas existentes</span>
                </li>
              </ul>
              <div className="text-center">
                <button className="btn-secondary w-full block cursor-pointer">
                  Contactar con ventas
                </button>
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="py-16 bg-button text-white">
        <div className="container-custom text-center">
          <h2 className="text-2xl md:text-3xl font-bold mb-6">¿Listo para experimentar AidGuide?</h2>
          <p className="max-w-2xl mx-auto mb-8 text-lg">
            Solicita una demostración gratuita y descubre cómo nuestro robot puede transformar la movilidad de personas
            invidentes.
          </p>
          <div className="flex flex-col sm:flex-row justify-center gap-4">
            <Link
              href="/register"
              className="bg-white text-button py-3 px-8 rounded-33 hover:bg-gray-100 transition-colors font-medium shadow-lg hover:shadow-xl transform hover:-translate-y-1"
            >
              Solicitar demostración
            </Link>
            <Link
              href="/contact"
              className="border border-white py-3 px-8 rounded-33 hover:bg-white/10 transition-colors font-medium"
            >
              Contactar con ventas
            </Link>
          </div>
        </div>
      </section>
    </div>
  )
}

