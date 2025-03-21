import Link from "next/link"
import Image from "next/image"

export default function Footer() {
  return (
    <footer className="bg-background-secondary text-white py-8">
      <div className="container-custom">
        <div className="grid grid-cols-1 md:grid-cols-4 gap-8">
          <div className="col-span-1 md:col-span-1">
            <Link href="/" className="flex items-center">
              <Image src="/logo.svg" alt="AidGuide Logo" width={80} height={60} className="h-10 w-auto" />
              <span className="ml-2 text-xl font-bold">AidGuide</span>
            </Link>
            <p className="mt-4 text-sm">
            Innovación que guía, accesibilidad que transforma.
            </p>
          </div>

          <div>
            <h4 className="font-semibold mb-4">Enlaces</h4>
            <ul className="space-y-2">

            {/*}  <li>
                <Link href="/" className="hover:underline">
                  Inicio
                </Link>
              </li>*/}

              <li>
                <Link href="/about" className="hover:underline">
                  Quiénes Somos
                </Link>
              </li>
              <li>
                <Link href="/product" className="hover:underline">
                  Producto
                </Link>
              </li>
              {/*<li>
                <Link href="/routes" className="hover:underline">
                  Rutas
                </Link>
              </li>*/}
              <li>
                <Link href="/faq" className="hover:underline">
                  FAQ
                </Link>
              </li>
            </ul>
          </div>

          <div>
            <h4 className="font-semibold mb-4">Soporte</h4>
            <ul className="space-y-2">
              
              <li>
                <Link href="/contact" className="hover:underline">
                  Contacto
                </Link>
              </li>
              <li>
                <Link href="/privacy" className="hover:underline">
                  Política de Privacidad
                </Link>
              </li>
              <li>
                <Link href="/terms" className="hover:underline">
                  Términos de Uso
                </Link>
              </li>
            </ul>
          </div>

          <div>
            <h4 className="font-semibold mb-4">Contacto</h4>
            <address className="not-italic">
              <p>Email: info@aidguide.com</p>
              <p>Teléfono: +34 123 456 789</p>
              <p>Dirección: C/ Paranimf, 123</p>
              <p>Gandía, Valencia, España</p>
            </address>
          </div>
        </div>

        <div className="border-t border-white/20 mt-8 pt-6 text-center text-sm">
          <p>&copy; {new Date().getFullYear()} AidGuide. Todos los derechos reservados.</p>
          <p className="mt-2">
            Desarrollado por: Vicente Rivas, Irene Medina, Mimi Vladeva, Hugo Belda, Marc Vilagrosa
          </p>
        </div>
      </div>
    </footer>
  )
}

