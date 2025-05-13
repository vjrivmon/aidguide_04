import { PageHeader } from "@/components/page-header";
import { PageHeaderDescription, PageHeaderHeading } from "@/components/page-header";
import ROS2Viewer from "../components/ROS2Viewer";

export default function ProcessingPage() {
  return (
    <div className="container relative pb-10">
      <PageHeader className="pb-8">
        <PageHeaderHeading>Procesamiento de Imágenes ROS2</PageHeaderHeading>
        <PageHeaderDescription>
          Inicia ROS2 con Gazebo y procesa imágenes en tiempo real utilizando algoritmos avanzados de visión por computadora.
        </PageHeaderDescription>
      </PageHeader>

      <div className="space-y-6">
        <ROS2Viewer />
      </div>
    </div>
  );
} 