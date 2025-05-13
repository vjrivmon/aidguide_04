#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Ejecutor Automático de Tests para AidGuide 04

Este script ejecuta todos los tests unitarios disponibles en el proyecto 
y genera un informe detallado de los resultados.

Author: AidGuide Team
"""

import os
import sys
import unittest
import time
import re
import datetime
from pathlib import Path
from termcolor import colored
import importlib.util

# Configuración
TEST_DIR = Path(__file__).parent
REPORT_FILE = TEST_DIR / "test_results.txt"

class TestRunner:
    """Ejecutor de tests unitarios con generación de informes."""
    
    def __init__(self):
        """Inicializa el ejecutor de tests."""
        self.test_files = []
        self.results = {}
        self.start_time = None
        self.end_time = None
        self.total_tests = 0
        self.passed_tests = 0
        self.failed_tests = 0
        self.skipped_tests = 0
        self.error_tests = 0
    
    def discover_tests(self):
        """Descubre todos los archivos de test en el directorio."""
        print(colored("🔍 Buscando archivos de test...", "cyan"))
        pattern = re.compile(r'^test_.*\.py$')
        
        for file in TEST_DIR.glob("*.py"):
            if pattern.match(file.name) and file.name != os.path.basename(__file__):
                self.test_files.append(file)
        
        print(colored(f"✅ Encontrados {len(self.test_files)} archivos de test", "green"))
        return self.test_files
    
    def run_test_file(self, test_file):
        """Ejecuta un archivo de test específico."""
        # Obtener nombre del módulo sin extensión
        module_name = test_file.stem
        
        # Cargar el módulo dinámicamente
        try:
            spec = importlib.util.spec_from_file_location(module_name, test_file)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            
            # Crear y ejecutar suite de pruebas
            suite = unittest.defaultTestLoader.loadTestsFromModule(module)
            result = unittest.TextTestRunner(verbosity=2).run(suite)
            
            # Recopilar resultados
            tests_run = result.testsRun
            failures = len(result.failures)
            errors = len(result.errors)
            skipped = len(result.skipped)
            passed = tests_run - failures - errors - skipped
            
            # Almacenar resultados
            self.results[test_file.name] = {
                'total': tests_run,
                'passed': passed,
                'failed': failures,
                'errors': errors,
                'skipped': skipped,
                'failures_details': result.failures,
                'errors_details': result.errors
            }
            
            # Actualizar contadores globales
            self.total_tests += tests_run
            self.passed_tests += passed
            self.failed_tests += failures
            self.error_tests += errors
            self.skipped_tests += skipped
            
            return True
        except Exception as e:
            print(colored(f"❌ Error al ejecutar {test_file}: {str(e)}", "red"))
            self.results[test_file.name] = {
                'total': 0,
                'passed': 0,
                'failed': 0,
                'errors': 1,
                'skipped': 0,
                'failures_details': [],
                'errors_details': [(None, str(e))]
            }
            self.error_tests += 1
            return False
    
    def run_all_tests(self):
        """Ejecuta todos los tests descubiertos."""
        if not self.test_files:
            self.discover_tests()
        
        if not self.test_files:
            print(colored("❌ No se encontraron archivos de test para ejecutar.", "red"))
            return False
        
        self.start_time = time.time()
        print(colored("\n🚀 Iniciando ejecución de tests...", "cyan"))
        
        for i, test_file in enumerate(sorted(self.test_files)):
            print(colored(f"\n[{i+1}/{len(self.test_files)}] Ejecutando {test_file.name}...", "cyan"))
            self.run_test_file(test_file)
        
        self.end_time = time.time()
        self.generate_report()
        return True
    
    def generate_report(self):
        """Genera un informe detallado de los resultados."""
        execution_time = self.end_time - self.start_time
        
        # Imprimir en consola
        print(colored("\n📊 RESUMEN DE RESULTADOS 📊", "cyan"))
        print(colored(f"Tests ejecutados: {self.total_tests}", "white"))
        print(colored(f"✅ Tests pasados: {self.passed_tests}", "green"))
        print(colored(f"❌ Tests fallidos: {self.failed_tests}", "red" if self.failed_tests > 0 else "white"))
        print(colored(f"⚠️ Tests con errores: {self.error_tests}", "red" if self.error_tests > 0 else "white"))
        print(colored(f"⏩ Tests omitidos: {self.skipped_tests}", "yellow" if self.skipped_tests > 0 else "white"))
        print(colored(f"⏱️ Tiempo de ejecución: {execution_time:.2f} segundos", "white"))
        
        # Guardar en archivo
        with open(REPORT_FILE, 'w', encoding='utf-8') as f:
            f.write(f"INFORME DE EJECUCIÓN DE TESTS - AidGuide 04\n")
            f.write(f"Fecha y hora: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("RESUMEN GENERAL\n")
            f.write("="*50 + "\n")
            f.write(f"Total de archivos de test: {len(self.test_files)}\n")
            f.write(f"Total de tests ejecutados: {self.total_tests}\n")
            f.write(f"Tests pasados: {self.passed_tests}\n")
            f.write(f"Tests fallidos: {self.failed_tests}\n")
            f.write(f"Tests con errores: {self.error_tests}\n")
            f.write(f"Tests omitidos: {self.skipped_tests}\n")
            f.write(f"Tiempo de ejecución: {execution_time:.2f} segundos\n\n")
            
            f.write("RESULTADOS POR ARCHIVO\n")
            f.write("="*50 + "\n")
            for filename, result in sorted(self.results.items()):
                f.write(f"\n{filename}\n")
                f.write("-"*len(filename) + "\n")
                f.write(f"Total: {result['total']}, ")
                f.write(f"Pasados: {result['passed']}, ")
                f.write(f"Fallidos: {result['failed']}, ")
                f.write(f"Errores: {result['errors']}, ")
                f.write(f"Omitidos: {result['skipped']}\n")
                
                # Detalles de fallos
                if result['failures_details']:
                    f.write("\nDetalles de fallos:\n")
                    for test_case, trace in result['failures_details']:
                        f.write(f"- {test_case}\n")
                        f.write(f"{trace}\n")
                
                # Detalles de errores
                if result['errors_details']:
                    f.write("\nDetalles de errores:\n")
                    for test_case, trace in result['errors_details']:
                        f.write(f"- {test_case if test_case else 'Error general'}\n")
                        f.write(f"{trace}\n")
        
        print(colored(f"\n📄 Informe detallado guardado en: {REPORT_FILE}", "green"))

def main():
    """Función principal."""
    print(colored("\n🤖 EJECUTOR DE TESTS UNITARIOS - AIDGUIDE 04 🤖\n", "cyan"))
    
    runner = TestRunner()
    runner.run_all_tests()
    
    # Mostrar mensaje de éxito o fracaso
    if runner.failed_tests > 0 or runner.error_tests > 0:
        print(colored("\n❌ ALGUNOS TESTS HAN FALLADO. Revisa el informe para más detalles.", "red"))
        return 1
    else:
        print(colored("\n✅ TODOS LOS TESTS HAN PASADO CORRECTAMENTE! 🎉", "green"))
        return 0

if __name__ == "__main__":
    sys.exit(main()) 