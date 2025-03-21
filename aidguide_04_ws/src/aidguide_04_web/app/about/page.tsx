import Image from "next/image"

export default function About() {
  const teamMembers = [
    {
      name: "Vicente Rivas Monferrer",
      role: "Scrum Master",
      bio: "Líder del proyecto con amplia experiencia en gestión de equipos y metodologías ágiles.",
      image: "/placeholder.svg?height=300&width=300",
    },
    {
      name: "Irene Medina García",
      role: "Desarrolladora",
      bio: "Especialista en desarrollo frontend y experiencia de usuario.",
      image: "/placeholder.svg?height=300&width=300",
    },
    {
      name: "Mimi Vladeva",
      role: "Desarrolladora",
      bio: "Experta en inteligencia artificial y procesamiento de visión en tiempo real.",
      image: "/placeholder.svg?height=300&width=300",
    },
    {
      name: "Hugo Belda Revert",
      role: "Desarrollador",
      bio: "Especialista en robótica y sistemas embebidos.",
      image: "/placeholder.svg?height=300&width=300",
    },
    {
      name: "Marc Vilagrosa Caturla",
      role: "Desarrollador",
      bio: "Experto en backend y arquitectura de sistemas.",
      image: "/placeholder.svg?height=300&width=300",
    },
  ]

  return (
    <div className="bg-background min-h-screen">
      {/* Hero Section */}
      <section className="py-16 md:py-24 bg-button text-white">
        <div className="container-custom">
          <div className="text-center">
            <h1 className="text-3xl md:text-4xl lg:text-5xl font-bold mb-6">Quiénes Somos</h1>
            <p className="text-lg max-w-3xl mx-auto">
              Somos un equipo comprometido con la creación de tecnologías inclusivas que mejoren la calidad de vida de
              las personas invidentes.
            </p>
          </div>
        </div>
      </section>

      {/* Mission & Vision */}
      <section className="py-16">
        <div className="container-custom">
          <div className="grid grid-cols-1 md:grid-cols-2 gap-12">
            <div className="card">
              <h2 className="text-2xl font-bold mb-4 text-button">Nuestra Misión</h2>
              <p className="mb-4">
                Desarrollar soluciones tecnológicas innovadoras que permitan a las personas invidentes desplazarse con
                mayor autonomía y seguridad en entornos urbanos.
              </p>
              <p>
                Buscamos eliminar barreras y crear un mundo más inclusivo donde la tecnología sea una herramienta de
                empoderamiento para todos.
              </p>
            </div>
            <div className="card">
              <h2 className="text-2xl font-bold mb-4 text-button">Nuestra Visión</h2>
              <p className="mb-4">
                Ser líderes en el desarrollo de tecnologías asistivas, reconocidos por la calidad, innovación y el
                impacto positivo de nuestras soluciones en la vida de las personas con discapacidad visual.
              </p>
              <p>
                Aspiramos a un futuro donde la movilidad independiente sea un derecho accesible para todos, sin importar
                sus capacidades visuales.
              </p>
            </div>
          </div>
        </div>
      </section>

      {/* Our Story */}
      <section className="py-16 bg-white">
        <div className="container-custom">
          <div className="text-center mb-12">
            <h2 className="text-2xl md:text-3xl font-bold mb-4">Nuestra Historia</h2>
            <p className="max-w-3xl mx-auto">El camino que nos ha llevado a desarrollar AidGuide.</p>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-2 gap-12 items-center">
            <div className="order-2 md:order-1">
              <h3 className="text-xl font-bold mb-4 text-button">De la idea a la realidad</h3>
              <p className="mb-4">
                AidGuide nació de la observación de las dificultades que enfrentan las personas invidentes en su
                movilidad diaria. Identificamos que, a pesar de los avances tecnológicos, seguían existiendo importantes
                barreras para su desplazamiento autónomo.
              </p>
              <p className="mb-4">
                Nuestro equipo, formado por profesionales de diversas disciplinas, se unió con el objetivo común de
                crear una solución integral que aprovechara las últimas tecnologías en inteligencia artificial, visión
                por computadora y robótica.
              </p>
              <p>
                Después de años de investigación, desarrollo y pruebas con usuarios reales, hemos creado un robot guía
                que no solo detecta obstáculos, sino que comprende el entorno y proporciona una asistencia personalizada
                y adaptativa.
              </p>
            </div>
            <div className="order-1 md:order-2 flex justify-center">
              <div className="relative w-full max-w-md h-80">
                <Image src="/logo.svg" alt="Historia de AidGuide" fill className="object-contain" /> 
              </div>
            </div>
          </div>
        </div>
      </section>

      {/* Team Section */}
      <section className="py-16">
        <div className="container-custom">
          <div className="text-center mb-12">
            <h2 className="text-2xl md:text-3xl font-bold mb-4">Nuestro Equipo</h2>
            <p className="max-w-3xl mx-auto">
              Conoce a las personas detrás de AidGuide, un equipo multidisciplinar comprometido con la innovación y la
              inclusión.
            </p>
          </div>

          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 gap-8">
            {teamMembers.map((member, index) => (
              <div key={index} className="card text-center">
                <div className="mb-4 mx-auto rounded-full overflow-hidden w-32 h-32 relative">
                  <Image src={member.image || "/placeholder.svg"} alt={member.name} fill className="object-cover" />
                </div>
                <h3 className="text-xl font-bold">{member.name}</h3>
                <p className="text-button font-medium mb-2">{member.role}</p>
                <p className="text-gray-600">{member.bio}</p>
              </div>
            ))}
          </div>
        </div>
      </section>

      {/* Values Section */}
      <section className="py-16 bg-white">
        <div className="container-custom">
          <div className="text-center mb-12">
            <h2 className="text-2xl md:text-3xl font-bold mb-4">Nuestros Valores</h2>
            <p className="max-w-3xl mx-auto">Los principios que guían nuestro trabajo y nuestra visión.</p>
          </div>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
            <div className="card">
              <h3 className="text-xl font-bold mb-4 text-button">Inclusión</h3>
              <p>
                Creemos en un mundo donde la tecnología elimine barreras en lugar de crearlas. Diseñamos pensando en la
                diversidad y en las necesidades reales de todos los usuarios.
              </p>
            </div>
            <div className="card">
              <h3 className="text-xl font-bold mb-4 text-button">Innovación</h3>
              <p>
                Buscamos constantemente nuevas formas de aplicar la tecnología para resolver problemas complejos. La
                creatividad y el pensamiento disruptivo son parte de nuestro ADN.
              </p>
            </div>
            <div className="card">
              <h3 className="text-xl font-bold mb-4 text-button">Compromiso</h3>
              <p>
                Nos comprometemos con la calidad, la seguridad y el impacto positivo de nuestras soluciones. Cada
                detalle importa cuando se trata de mejorar la vida de las personas.
              </p>
            </div>
          </div>
        </div>
      </section>
    </div>
  )
}

