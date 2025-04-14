#!/usr/bin/env python3
"""
Nodo puente entre la interfaz web y el seguidor de waypoints
Autor: Claude
Fecha: 2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import threading
import subprocess
import signal
import os
import time
from rclpy.executors import MultiThreadedExecutor

class WebWaypointBridge(Node):
    """
    Nodo que actúa como puente entre mensajes web y el seguidor de waypoints
    """
    def __init__(self):
        """Función Constructor.
        """
        super().__init__('web_waypoint_bridge')
        
        # Suscriptores para mensajes desde la web
        self.start_sub = self.create_subscription(
            Bool,
            '/start_waypoint_following',
            self.start_waypoint_callback,
            10
        )
        
        self.stop_sub = self.create_subscription(
            Bool,
            '/stop_waypoint_following',
            self.stop_waypoint_callback,
            10
        )
        
        # Publicadores para el estado
        self.status_pub = self.create_publisher(
            String,
            '/navigation_status',
            10
        )
        
        # Variables de estado
        self.navigation_process = None
        self.is_running = False
        self.status_thread = None
        self.stop_thread_flag = False
        self.manually_stopped = False  # Nueva bandera para indicar si fue detenido manualmente
        
        self.get_logger().info('Nodo puente de waypoints inicializado')
        self.publish_status("Listo para navegar")

    def start_waypoint_callback(self, msg):
        """Callback para iniciar la navegación"""
        if not msg.data:
            return
        
        if self.is_running:
            self.get_logger().warning('La navegación ya está en marcha. Ignorando solicitud.')
            return
        
        self.get_logger().info('Iniciando seguimiento de waypoints por solicitud web')
        
        # Iniciar navegación
        self.start_navigation()
        
    def stop_waypoint_callback(self, msg):
        """Callback para detener la navegación"""
        if not msg.data:
            return
        
        if not self.is_running:
            self.get_logger().warning('La navegación no está activa. Ignorando solicitud de detención.')
            return
        
        self.get_logger().info('Deteniendo seguimiento de waypoints por solicitud web')
        
        # Establecer la bandera de detención manual
        self.manually_stopped = True
        
        # Detener navegación
        self.stop_navigation()
        
    def start_navigation(self):
        """Iniciar el nodo seguidor de waypoints"""
        if self.is_running:
            return
        
        # Restablecer la bandera de detención manual cuando iniciamos manualmente
        self.manually_stopped = False
        
        self.get_logger().info('Lanzando proceso de navegación')
        
        try:
            # Lanzar el proceso del waypoint follower
            self.navigation_process = subprocess.Popen(
                ['ros2', 'run', 'aidguide_04_my_nav2_system', 'my_waypoint_follower'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid
            )
            
            self.is_running = True
            self.publish_status("Navegación iniciada")
            
            # Iniciar un hilo para monitorizar el estado
            self.stop_thread_flag = False
            self.status_thread = threading.Thread(target=self.monitor_navigation)
            self.status_thread.daemon = True
            self.status_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Error al iniciar navegación: {str(e)}')
            self.publish_status(f"Error: {str(e)}")
            self.is_running = False
    
    def stop_navigation(self):
        """Detener el nodo seguidor de waypoints"""
        if not self.is_running:
            return
        
        self.get_logger().info('Deteniendo proceso de navegación')
        
        try:
            # Detener el hilo de monitorización
            self.stop_thread_flag = True
            if self.status_thread:
                self.status_thread.join(timeout=1.0)
                
            # Terminar el proceso
            if self.navigation_process:
                os.killpg(os.getpgid(self.navigation_process.pid), signal.SIGTERM)
                self.navigation_process.wait(timeout=5.0)
                self.navigation_process = None
                
            self.is_running = False
            
            # Mensaje diferente según si fue detenido manualmente o no
            if self.manually_stopped:
                self.publish_status("Navegación detenida por el usuario")
            else:
                self.publish_status("Navegación detenida")
            
        except Exception as e:
            self.get_logger().error(f'Error al detener navegación: {str(e)}')
            self.publish_status(f"Error al detener: {str(e)}")
            
            # Intentar forzar la terminación
            try:
                if self.navigation_process:
                    os.killpg(os.getpgid(self.navigation_process.pid), signal.SIGKILL)
                    self.navigation_process = None
            except:
                pass
            
            self.is_running = False
    
    def monitor_navigation(self):
        """Monitorizar el estado del proceso de navegación"""
        if not self.navigation_process:
            return
            
        while not self.stop_thread_flag:
            # Verificar si el proceso sigue activo
            if self.navigation_process.poll() is not None:
                # El proceso terminó
                exitcode = self.navigation_process.returncode
                self.get_logger().info(f'El proceso de navegación finalizó con código {exitcode}')
                
                # Leer cualquier mensaje de error
                stderr = self.navigation_process.stderr.read()
                if stderr:
                    self.get_logger().error(f'Error en el proceso: {stderr}')
                
                # Actualizar estado
                self.is_running = False
                
                # Sólo actualizamos el estado si no fue detenido manualmente
                if not self.manually_stopped:
                    if exitcode == 0:
                        self.publish_status("Navegación completada con éxito")
                    else:
                        self.publish_status(f"Error en navegación (código {exitcode})")
                
                # Limpiar
                self.navigation_process = None
                break
            
            # Leer salida si hay disponible (sin bloquear)
            try:
                output = self.navigation_process.stdout.readline()
                if output:
                    # Buscar mensajes clave en la salida para actualizar el estado
                    if "Enviando waypoints al servidor" in output:
                        self.publish_status("Enviando waypoints...")
                    elif "Goal accepted by server" in output:
                        self.publish_status("Ruta aceptada, iniciando navegación")
                    elif "Currently at waypoint" in output:
                        waypoint = output.split(":")[-1].strip()
                        self.publish_status(f"Navegando: waypoint {waypoint}")
                    elif "Result received" in output:
                        self.publish_status("Recibiendo resultados...")
            except:
                pass
                
            time.sleep(0.1)
    
    def publish_status(self, status_text):
        """Publicar mensaje de estado para la interfaz web"""
        msg = String()
        msg.data = status_text
        self.status_pub.publish(msg)
        self.get_logger().info(f'Estado: {status_text}')
    
    def destroy_node(self):
        """Limpiar recursos al finalizar el nodo"""
        self.stop_navigation()
        super().destroy_node()


def main(args=None):
    """Función Main.
    
    Args:
        args (Any): Descripción del parámetro.
    """
    rclpy.init(args=args)
    
    bridge_node = WebWaypointBridge()
    
    # Usar un executor multihilo para manejar callbacks y monitoreo del proceso
    executor = MultiThreadedExecutor()
    executor.add_node(bridge_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 