"use client"

import { useState } from 'react'
import { FaEnvelope, FaPhone, FaMapMarkerAlt, FaClock } from 'react-icons/fa'

export default function Contact() {
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    subject: '',
    message: ''
  })

  const [status, setStatus] = useState({
    type: '',
    message: ''
  })

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target
    setFormData(prevState => ({
      ...prevState,
      [name]: value
    }))
  }

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault()
    setStatus({ type: 'info', message: 'Enviando mensaje...' })

    try {
      await new Promise(resolve => setTimeout(resolve, 1000))
      setStatus({
        type: 'success',
        message: 'Mensaje enviado correctamente. Nos pondremos en contacto contigo pronto.'
      })
      setFormData({ name: '', email: '', subject: '', message: '' })
    } catch (error) {
      setStatus({
        type: 'error',
        message: 'Hubo un error al enviar el mensaje. Por favor, inténtalo de nuevo.'
      })
    }
  }

  return (
    <div className="min-h-[calc(100vh-200px)] py-12 px-4 sm:px-6 lg:px-8 bg-background">
      <div className="max-w-7xl mx-auto">
        <div className="text-center mb-12">
          <h1 className="text-4xl font-bold text-gray-900 mb-4">Contacta con Nosotros</h1>
          <p className="text-lg text-gray-600">
            Estamos aquí para ayudarte. Envíanos tus dudas o comentarios.
          </p>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-3 gap-8">
          {/* Información de contacto */}
          <div className="lg:col-span-1">
            <div className="bg-white rounded-33 shadow-md p-8">
              <h2 className="text-2xl font-semibold mb-6 text-button">Información de Contacto</h2>
              
              <div className="space-y-6">
                <div className="flex items-start space-x-4">
                  <div className="text-button mt-1">
                    <FaMapMarkerAlt size={24} />
                  </div>
                  <div>
                    <h3 className="font-semibold mb-1">Dirección</h3>
                    <p className="text-gray-600">
                      C/ Paranimf, 1<br />
                      46730 Grau de Gandia, Valencia
                    </p>
                  </div>
                </div>

                <div className="flex items-start space-x-4">
                  <div className="text-button mt-1">
                    <FaEnvelope size={24} />
                  </div>
                  <div>
                    <h3 className="font-semibold mb-1">Email</h3>
                    <p className="text-gray-600">info@aidguide.com</p>
                  </div>
                </div>

                <div className="flex items-start space-x-4">
                  <div className="text-button mt-1">
                    <FaPhone size={24} />
                  </div>
                  <div>
                    <h3 className="font-semibold mb-1">Teléfono</h3>
                    <p className="text-gray-600">+34 123 456 789</p>
                  </div>
                </div>

                <div className="flex items-start space-x-4">
                  <div className="text-button mt-1">
                    <FaClock size={24} />
                  </div>
                  <div>
                    <h3 className="font-semibold mb-1">Horario</h3>
                    <p className="text-gray-600">
                      Lun - Vie: 9:00 - 18:00<br />
                      Sáb: 10:00 - 14:00
                    </p>
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Formulario de contacto */}
          <div className="lg:col-span-2">
            <div className="bg-white rounded-33 shadow-md p-8">
              <h2 className="text-2xl font-semibold mb-6 text-button">Envíanos un Mensaje</h2>
              
              <form onSubmit={handleSubmit} className="space-y-6">
                <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                  <div>
                    <label htmlFor="name" className="block text-sm font-medium text-gray-700 mb-2">
                      Nombre completo *
                    </label>
                    <input
                      type="text"
                      id="name"
                      name="name"
                      value={formData.name}
                      onChange={handleChange}
                      required
                      className="w-full px-4 py-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-button focus:border-transparent transition-colors duration-200"
                      placeholder="Tu nombre"
                    />
                  </div>

                  <div>
                    <label htmlFor="email" className="block text-sm font-medium text-gray-700 mb-2">
                      Email *
                    </label>
                    <input
                      type="email"
                      id="email"
                      name="email"
                      value={formData.email}
                      onChange={handleChange}
                      required
                      className="w-full px-4 py-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-button focus:border-transparent transition-colors duration-200"
                      placeholder="tu@email.com"
                    />
                  </div>
                </div>

                <div>
                  <label htmlFor="subject" className="block text-sm font-medium text-gray-700 mb-2">
                    Asunto *
                  </label>
                  <input
                    type="text"
                    id="subject"
                    name="subject"
                    value={formData.subject}
                    onChange={handleChange}
                    required
                    className="w-full px-4 py-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-button focus:border-transparent transition-colors duration-200"
                    placeholder="Asunto del mensaje"
                  />
                </div>

                <div>
                  <label htmlFor="message" className="block text-sm font-medium text-gray-700 mb-2">
                    Mensaje *
                  </label>
                  <textarea
                    id="message"
                    name="message"
                    value={formData.message}
                    onChange={handleChange}
                    required
                    rows={6}
                    className="w-full px-4 py-3 border border-gray-300 rounded-md focus:outline-none focus:ring-2 focus:ring-button focus:border-transparent transition-colors duration-200"
                    placeholder="¿En qué podemos ayudarte?"
                  />
                </div>

                {status.message && (
                  <div className={`p-4 rounded-md ${
                    status.type === 'success' ? 'bg-green-100 text-green-700' :
                    status.type === 'error' ? 'bg-red-100 text-red-700' :
                    'bg-blue-100 text-blue-700'
                  }`}>
                    {status.message}
                  </div>
                )}

                <button
                  type="submit"
                  className="w-full bg-button text-white py-3 px-6 rounded-md hover:bg-opacity-90 transition-colors duration-200 font-medium text-lg"
                >
                  Enviar mensaje
                </button>
              </form>
            </div>
          </div>
        </div>

        {/* Mapa */}
        <div className="mt-12">
          <div className="bg-white rounded-33 shadow-md p-8">
            <h2 className="text-2xl font-semibold mb-6 text-button">Nuestra Ubicación</h2>
            <div className="w-full h-96 rounded-md overflow-hidden">
              <iframe
                src="https://www.google.com/maps/embed?pb=!1m18!1m12!1m3!1d3111.0300618075275!2d-0.16669492424931817!3d38.99559994388561!2m3!1f0!2f0!3f0!3m2!1i1024!2i768!4f13.1!3m3!1m2!1s0xd61b87a1d29d379%3A0x5aec5c00a81b1439!2sUPV%20Campus%20Gandia%20-%20Universitat%20Polit%C3%A8cnica%20de%20Val%C3%A8ncia!5e0!3m2!1ses!2ses!4v1711059789744!5m2!1ses!2ses"
                width="100%"
                height="100%"
                style={{ border: 0 }}
                allowFullScreen
                loading="lazy"
                referrerPolicy="no-referrer-when-downgrade"
              />
            </div>
          </div>
        </div>
      </div>
    </div>
  )
} 