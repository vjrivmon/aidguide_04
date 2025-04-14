#!/usr/bin/env python3
"""
Sistema de Pruebas Automatizadas para AidGuide 04.

Este módulo implementa pruebas automatizadas para verificar la funcionalidad
del sistema AidGuide 04, incluyendo la conexión con ROS2, simulación y navegación.

Author: AidGuide Team
Date: 2024-03-30
"""

import os
import sys
import time
import logging
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from typing import List, Tuple, Optional, Dict, Any
import socket

# Configuración del logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class TestConfig:
    """Configuración global para las pruebas."""
    
    REQUIRED_FRAMES = [
        'map',
        'odom',
        'base_link',
        'base_footprint',
        'laser_frame'
    ]
    
    REQUIRED_TERMINALS = [
        'ros2',
        'gazebo',
        'nav2',
        'rviz2'
    ]
    
    TIMEOUT = 5.0

    TEST_CASES = [
        "Conexión con ROS2",
        "Verificar mapa cargado",
        "Verificar robot en simulación",
        "Verificar navegación",
        "Verificar interfaz web"
    ]

class TestResult:
    """Resultado de una prueba individual."""
    
    def __init__(self, name: str):
        """
        Inicializa un resultado de prueba.
        
        Args:
            name: Nombre de la prueba
        """
        self.name = name
        self.status = "Pendiente"
        self.details = ""
        self.passed = None

class TestUtils:
    """Utilidades para las pruebas."""
    
    @staticmethod
    def verify_terminal(name: str) -> bool:
        """
        Verifica si un terminal específico está en ejecución.
        
        Args:
            name: Nombre del terminal a verificar
            
        Returns:
            bool: True si el terminal está en ejecución, False en caso contrario
        """
        try:
            result = subprocess.run(
                ['pgrep', '-f', name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            return result.returncode == 0
        except Exception as e:
            logger.error(f"Error al verificar terminal {name}: {e}")
            return False
    
    @staticmethod
    def verify_tf_transform(frame_origin: str, frame_target: str, timeout: float = 0.5) -> bool:
        """
        Verifica la existencia de una transformada TF entre dos frames.

        Args:
            frame_origin: Frame de origen
            frame_target: Frame de destino
            timeout: Tiempo máximo de espera en segundos

        Returns:
            bool: True si la transformada existe, False en caso contrario
        """
        try:
            result = subprocess.run(
                ['ros2', 'run', 'tf2_ros', 'tf2_echo', frame_origin, frame_target],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=timeout
            )
            return "Transform" in result.stdout
        except subprocess.TimeoutExpired:
            logger.warning(f"Timeout al verificar transformada {frame_origin} -> {frame_target}")
            return False
        except Exception as e:
            logger.error(f"Error al verificar transformada: {e}")
            return False

    @staticmethod
    def get_available_frames() -> List[str]:
        """
        Obtiene la lista de frames TF disponibles.

        Returns:
            List[str]: Lista de frames TF disponibles
        """
        try:
            result = subprocess.run(
                ['ros2', 'run', 'tf2_tools', 'tf2_monitor', '--once'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=TestConfig.TIMEOUT
            )
            frames = []
            for line in result.stdout.split('\n'):
                if 'Frame' in line and '->' in line:
                    frames.extend(line.split('->')[0].strip().split())
            return list(set(frames))
        except Exception as e:
            logger.error(f"Error al obtener frames TF: {e}")
            return []

class TestGUI(tk.Tk):
    """Interfaz gráfica para las pruebas automatizadas."""
    
    def __init__(self):
        """Inicializa la interfaz gráfica."""
        super().__init__()
        self.title("Pruebas Automatizadas AidGuide 04")
        self.geometry("800x600")
        self.configure(bg="#f0f0f0")
        
        self.logger = logging.getLogger(__name__ + '.TestGUI')
        self.test_results = [TestResult(test) for test in TestConfig.TEST_CASES]
        self.running = False
        self.tests_completed = 0
        self.tests_passed = 0
        
        self._setup_styles()
        self._create_interface()
    
    def _setup_styles(self):
        """Configura los estilos de la interfaz."""
        self.style = ttk.Style()
        self.style.configure("TLabel", font=("Arial", 12), background="#f0f0f0")
        self.style.configure("Header.TLabel", font=("Arial", 16, "bold"), background="#f0f0f0")
    
    def _create_interface(self):
        """Crea los elementos de la interfaz gráfica."""
        # Panel de título
        title_frame = ttk.Frame(self)
        title_frame.pack(fill="x", padx=20, pady=20)
        
        ttk.Label(title_frame, text="Sistema de Pruebas Automatizadas AidGuide 04", 
                style="Header.TLabel").pack()
        
        # Panel de información
        info_frame = ttk.Frame(self)
        info_frame.pack(fill="x", padx=20, pady=10)
        
        self.connection_label = ttk.Label(info_frame, text="Estado de la conexión: No conectado")
        self.connection_label.pack(anchor="w")
        
        # Panel de pruebas
        self.tests_frame = ttk.LabelFrame(self, text="Pruebas de Funcionalidad")
        self.tests_frame.pack(fill="both", expand=True, padx=20, pady=10)
        
        # Crear checkboxes para cada prueba
        self.test_vars = []
        for i, result in enumerate(self.test_results):
            var = tk.IntVar()
            cb = ttk.Checkbutton(self.tests_frame, text=f"{i+1}. {result.name}", 
                                variable=var, state="disabled")
            cb.pack(anchor="w", padx=20, pady=5)
            self.test_vars.append((var, cb))
        
        # Panel de registro
        log_frame = ttk.LabelFrame(self, text="Registro de eventos")
        log_frame.pack(fill="both", expand=True, padx=20, pady=10)
        
        self.log_text = tk.Text(log_frame, height=10, width=80)
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        self.log_text.config(state="disabled")
        
        # Panel de botones
        buttons_frame = ttk.Frame(self)
        buttons_frame.pack(fill="x", padx=20, pady=20)
        
        self.start_button = ttk.Button(buttons_frame, text="Iniciar Pruebas", 
                                    command=self.start_tests)
        self.start_button.pack(side="left", padx=5)
        
        self.stop_button = ttk.Button(buttons_frame, text="Detener", 
                                    command=self.stop_tests, state="disabled")
        self.stop_button.pack(side="left", padx=5)
        
        # Barra de progreso
        self.progress = ttk.Progressbar(buttons_frame, length=400, mode="determinate")
        self.progress.pack(side="right", padx=5)
    
    def log_message(self, message: str, level: str = "INFO"):
        """
        Registra un mensaje en el panel de log.

        Args:
            message: Mensaje a registrar
            level: Nivel del mensaje (INFO, WARNING, ERROR)
        """
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")
        
        # También registrar en el sistema de logging
        if level == "ERROR":
            self.logger.error(message)
        elif level == "WARNING":
            self.logger.warning(message)
        else:
            self.logger.info(message)

    def update_test_status(self, index: int, status: str, details: str = "", passed: bool = None):
        """
        Actualiza el estado de una prueba.

        Args:
            index: Índice de la prueba
            status: Estado de la prueba
            details: Detalles adicionales
            passed: Si la prueba pasó o no
        """
        self.test_results[index].status = status
        self.test_results[index].details = details
        
        if passed is not None:
            self.test_results[index].passed = passed
            var, cb = self.test_vars[index]
            var.set(1 if passed else 0)
            
            # Actualizar progreso
            self.tests_completed += 1
            if passed:
                self.tests_passed += 1
            
            # Actualizar barra de progreso
            self.progress["value"] = (self.tests_completed / len(self.test_results)) * 100
            
            # Actualizar UI
            self.update_idletasks()
    
    def start_tests(self):
        """Inicia la secuencia de pruebas."""
        self.running = True
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        
        # Resetear estado
        self.tests_completed = 0
        self.tests_passed = 0
        self.progress["value"] = 0
        
        # Resetear checkboxes
        for var, cb in self.test_vars:
            var.set(0)
            cb.config(state="normal")
        
        # Limpiar log
        self.log_text.config(state="normal")
        self.log_text.delete(1.0, "end")
        self.log_text.config(state="disabled")
        
        # Iniciar pruebas en un hilo separado
        self.test_thread = threading.Thread(target=self.run_tests)
        self.test_thread.daemon = True
        self.test_thread.start()
    
    def stop_tests(self):
        """Detiene la ejecución de pruebas."""
        self.running = False
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.log_message("Pruebas detenidas por el usuario")
    
    def run_tests(self):
        """Ejecuta la secuencia completa de pruebas."""
        self.log_message("Iniciando secuencia de pruebas automatizadas")
        
        try:
            # Test 1: Verificar conexión con ROS2
            self.test_ros2_connection()
            if not self.running:
                return
            
            time.sleep(1)
            
            # Test 2: Verificar mapa cargado
            self.test_map_loaded()
            if not self.running:
                return
            
            time.sleep(1)
            
            # Test 3: Verificar robot en simulación
            self.test_robot_in_simulation()
            if not self.running:
                return
            
            time.sleep(1)
            
            # Test 4: Verificar navegación
            self.test_navigation_system()
            if not self.running:
                return
            
            time.sleep(1)
            
            # Test 5: Verificar interfaz web
            self.test_web_interface()
            if not self.running:
                return
            
            # Finalizar pruebas
            self.log_message(f"Pruebas completadas: {self.tests_completed}/{len(self.test_results)}")
            self.log_message(f"Pruebas exitosas: {self.tests_passed}/{len(self.test_results)}")
            
            if self.tests_passed == len(self.test_results):
                messagebox.showinfo("Éxito", "¡Todas las pruebas completadas con éxito!")
            else:
                messagebox.showwarning("Advertencia", 
                                    f"{self.tests_passed} de {len(self.test_results)} pruebas exitosas.\n"
                                    "Revisa el registro para más detalles.")
            
        except Exception as e:
            self.log_message(f"Error durante las pruebas: {str(e)}", "ERROR")
            messagebox.showerror("Error", f"Se produjo un error durante las pruebas: {str(e)}")
        finally:
            self.running = False
            self.start_button.config(state="normal")
            self.stop_button.config(state="disabled")
    
    def test_ros2_connection(self):
        """Prueba 1: Verificar conexión con ROS2."""
        self.log_message("Prueba 1: Verificando conexión con ROS2...")
        self.update_test_status(0, "En progreso")
        
        try:
            result = subprocess.run(
                ["ros2", "topic", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=TestConfig.TIMEOUT
            )
            
            if result.returncode == 0 and result.stdout:
                self.log_message("✅ Conexión con ROS2 establecida")
                self.log_message(f"Tópicos disponibles: {len(result.stdout.splitlines())}")
                
                # Verificar terminales requeridos
                active_terminals = []
                missing_terminals = []
                
                for terminal in TestConfig.REQUIRED_TERMINALS:
                    if TestUtils.verify_terminal(terminal):
                        active_terminals.append(terminal)
                    else:
                        missing_terminals.append(terminal)
                
                if active_terminals:
                    self.log_message("✅ Terminales activos detectados:")
                    for terminal in active_terminals:
                        self.log_message(f"  • {terminal}")
                
                if missing_terminals:
                    self.log_message("⚠️ Terminales faltantes:")
                    for terminal in missing_terminals:
                        self.log_message(f"  • {terminal}")
                
                self.update_test_status(0, "Éxito", "Conexión con ROS2 establecida", True)
                self.connection_label.config(text="Estado de la conexión: Conectado")
            else:
                self.log_message("❌ No se pudo conectar con ROS2")
                self.log_message("Por favor, asegúrate de que ROS2 está en ejecución")
                self.update_test_status(0, "Fallo", "No se pudo conectar con ROS2", False)
                messagebox.showerror("Error", "No se pudo conectar con ROS2.\nAsegúrate de haber ejecutado start-ros2-gazebo.sh primero.")
                self.stop_tests()
                return
        except Exception as e:
            self.log_message(f"❌ Error al conectar con ROS2: {str(e)}", "ERROR")
            self.update_test_status(0, "Fallo", f"Error: {str(e)}", False)
            self.stop_tests()
    
    def test_map_loaded(self):
        """Prueba 2: Verificar mapa cargado."""
        self.log_message("Prueba 2: Verificando mapa cargado...")
        self.update_test_status(1, "En progreso")
        
        try:
            # Verificar si el servicio de mapa está disponible
            result = subprocess.run(
                ["ros2", "service", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=TestConfig.TIMEOUT
            )
            
            if "/map_server/load_map" in result.stdout:
                self.log_message("✅ Servicio de mapa encontrado")
                
                # Verificar si hay un tópico de mapa publicado
                result = subprocess.run(
                    ["ros2", "topic", "list"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    timeout=TestConfig.TIMEOUT
                )
                
                if "/map" in result.stdout:
                    self.log_message("✅ Tópico de mapa publicado correctamente")
                    self.update_test_status(1, "Éxito", "Mapa cargado correctamente", True)
                else:
                    self.log_message("⚠️ Servicio de mapa disponible pero no se detecta el tópico /map")
                    self.update_test_status(1, "Fallo", "No se detecta el tópico /map", False)
            else:
                self.log_message("❌ Servicio de mapa no encontrado")
                self.update_test_status(1, "Fallo", "Servicio de mapa no encontrado", False)
        except Exception as e:
            self.log_message(f"❌ Error al verificar mapa: {str(e)}", "ERROR")
            self.update_test_status(1, "Fallo", f"Error: {str(e)}", False)
    
    def test_robot_in_simulation(self):
        """Prueba 3: Verificar robot en simulación."""
        self.log_message("Prueba 3: Verificando robot en simulación...")
        self.update_test_status(2, "En progreso")
        
        try:
            # Obtener lista de frames TF disponibles
            frames_available = TestUtils.get_available_frames()
            
            if frames_available:
                self.log_message("✅ Frames TF detectados:")
                for frame in frames_available:
                    self.log_message(f"  • {frame}")
                
                # Verificar frames requeridos
                missing_frames = [frame for frame in TestConfig.REQUIRED_FRAMES 
                               if frame not in frames_available]
                
                if missing_frames:
                    self.log_message("⚠️ Frames TF faltantes:")
                    for frame in missing_frames:
                        self.log_message(f"  • {frame}")
                    self.update_test_status(2, "Fallo", "Faltan frames TF requeridos", False)
                else:
                    self.log_message("✅ Todos los frames TF requeridos están presentes")
                    
                    # Verificar transformadas clave
                    valid_tf = True
                    self.log_message("Verificando transformadas principales...")
                    
                    # Verificar map -> base_link
                    if TestUtils.verify_tf_transform('map', 'base_link'):
                        self.log_message("✅ Transformada map -> base_link válida")
                    else:
                        self.log_message("❌ No se detectó transformada map -> base_link")
                        valid_tf = False
                    
                    # Verificar odom -> base_link
                    if TestUtils.verify_tf_transform('odom', 'base_link'):
                        self.log_message("✅ Transformada odom -> base_link válida")
                    else:
                        self.log_message("❌ No se detectó transformada odom -> base_link")
                        valid_tf = False
                    
                    # Verificar base_link -> laser_frame
                    if TestUtils.verify_tf_transform('base_link', 'laser_frame'):
                        self.log_message("✅ Transformada base_link -> laser_frame válida")
                    else:
                        self.log_message("❌ No se detectó transformada base_link -> laser_frame")
                        valid_tf = False
                    
                    self.update_test_status(2, "Éxito" if valid_tf else "Fallo", 
                                         "Transformadas TF válidas" if valid_tf else "Transformadas TF inválidas", 
                                         valid_tf)
            else:
                self.log_message("❌ No se detectaron frames TF")
                self.update_test_status(2, "Fallo", "No se detectaron frames TF", False)
        except Exception as e:
            self.log_message(f"❌ Error al verificar robot: {str(e)}", "ERROR")
            self.update_test_status(2, "Fallo", f"Error: {str(e)}", False)
    
    def test_navigation_system(self):
        """Prueba 4: Verificar sistema de navegación."""
        self.log_message("Prueba 4: Verificando sistema de navegación...")
        self.update_test_status(3, "En progreso")
        
        try:
            # Verificar si los nodos de navegación están activos
            result = subprocess.run(
                ["ros2", "node", "list"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=TestConfig.TIMEOUT
            )
            
            nav_nodes = [node for node in result.stdout.splitlines() 
                         if "nav" in node.lower() or "planner" in node.lower() 
                         or "controller" in node.lower()]
            
            if nav_nodes:
                self.log_message(f"✅ {len(nav_nodes)} nodos de navegación detectados")
                self.log_message(f"Ejemplo: {nav_nodes[0]}")
                self.update_test_status(3, "Éxito", f"{len(nav_nodes)} nodos de navegación detectados", True)
            else:
                self.log_message("❌ No se detectaron nodos de navegación")
                self.update_test_status(3, "Fallo", "No se detectaron nodos de navegación", False)
        except Exception as e:
            self.log_message(f"❌ Error al verificar navegación: {str(e)}", "ERROR")
            self.update_test_status(3, "Fallo", f"Error: {str(e)}", False)
    
    def test_web_interface(self):
        """Prueba 5: Verificar interfaz web."""
        self.log_message("Prueba 5: Verificando interfaz web...")
        self.update_test_status(4, "En progreso")
        
        try:
            # Intentar conectar al puerto 3000 donde se ejecuta la interfaz web
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2)
            
            try:
                s.connect(('localhost', 3000))
                self.log_message("✅ Interfaz web disponible en http://localhost:3000")
                self.update_test_status(4, "Éxito", "Interfaz web disponible en http://localhost:3000", True)
            except:
                self.log_message("⚠️ No se detectó la interfaz web en el puerto 3000")
                self.log_message("Puedes iniciarla con: ./start-frontend.sh")
                self.update_test_status(4, "Fallo", "No se detectó la interfaz web", False)
            finally:
                s.close()
        except Exception as e:
            self.log_message(f"❌ Error al verificar interfaz web: {str(e)}", "ERROR")
            self.update_test_status(4, "Fallo", f"Error: {str(e)}", False)

def main():
    """Función principal del sistema de pruebas."""
    try:
        gui = TestGUI()
        # Iniciar pruebas automáticamente
        gui.after(500, gui.start_tests)
        gui.mainloop()
        
        # Si llegamos aquí con pruebas exitosas, retornar éxito
        if gui.tests_passed == len(gui.test_results):
            return 0
        else:
            return 1
    except Exception as e:
        logging.error(f"Error en la aplicación: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 