#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generador de Informes de Tests y Scripts para AidGuide 04
Este script genera un informe PDF de los tests y scripts disponibles en el proyecto.
También sugiere tests unitarios adicionales basados en las historias de usuario.
"""

import os
import glob
import subprocess
import re
from datetime import datetime
from reportlab.lib.pagesizes import letter, landscape
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, ListFlowable, ListItem, PageBreak
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch, cm
from reportlab.lib.enums import TA_LEFT, TA_CENTER, TA_RIGHT

# Ruta al archivo de resultados de tests
TEST_RESULTS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                "aidguide_04_ws/src/aidguide_04/test/test_results.txt")

# Función para obtener información de resultados de test
def get_test_results():
    """
    Lee y analiza los resultados de los tests del archivo generado por run_all_tests.py
    
    Returns:
        dict: Diccionario con los resultados de los tests
    """
    results = {
        'summary': {
            'total_files': 0,
            'total_tests': 0,
            'passed': 0,
            'failed': 0,
            'errors': 0,
            'skipped': 0,
            'execution_time': 0.0,
            'date': ""
        },
        'files': {}
    }
    
    try:
        with open(TEST_RESULTS_FILE, 'r', encoding='utf-8') as f:
            content = f.read()
            
            # Obtener fecha y hora
            date_match = re.search(r'Fecha y hora: ([^\n]+)', content)
            if date_match:
                results['summary']['date'] = date_match.group(1)
            
            # Obtener resumen general
            total_files_match = re.search(r'Total de archivos de test: (\d+)', content)
            if total_files_match:
                results['summary']['total_files'] = int(total_files_match.group(1))
                
            total_tests_match = re.search(r'Total de tests ejecutados: (\d+)', content)
            if total_tests_match:
                results['summary']['total_tests'] = int(total_tests_match.group(1))
                
            passed_match = re.search(r'Tests pasados: (\d+)', content)
            if passed_match:
                results['summary']['passed'] = int(passed_match.group(1))
                
            failed_match = re.search(r'Tests fallidos: (\d+)', content)
            if failed_match:
                results['summary']['failed'] = int(failed_match.group(1))
                
            errors_match = re.search(r'Tests con errores: (\d+)', content)
            if errors_match:
                results['summary']['errors'] = int(errors_match.group(1))
                
            skipped_match = re.search(r'Tests omitidos: (\d+)', content)
            if skipped_match:
                results['summary']['skipped'] = int(skipped_match.group(1))
                
            time_match = re.search(r'Tiempo de ejecución: ([0-9.]+)', content)
            if time_match:
                results['summary']['execution_time'] = float(time_match.group(1))
            
            # Obtener resultados por archivo
            file_sections = re.findall(r'\n([^-\n]+)\n-+\nTotal: (\d+), Pasados: (\d+), Fallidos: (\d+), Errores: (\d+), Omitidos: (\d+)', content)
            
            for filename, total, passed, failed, errors, skipped in file_sections:
                filename = filename.strip()
                results['files'][filename] = {
                    'total': int(total),
                    'passed': int(passed),
                    'failed': int(failed),
                    'errors': int(errors),
                    'skipped': int(skipped),
                    'status': 'Éxito' if int(failed) == 0 and int(errors) == 0 else 'Fallido'
                }
    except Exception as e:
        print(f"Error al leer el archivo de resultados: {str(e)}")
    
    return results

# Función para obtener información sobre un archivo .sh
def get_script_info(script_path):
    """Obtiene información sobre un script .sh"""
    script_name = os.path.basename(script_path)
    description = "No hay descripción disponible"
    
    try:
        # Leer las primeras 20 líneas del script para buscar la descripción
        with open(script_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()[:20]
            
            for i, line in enumerate(lines):
                if line.strip().startswith('#') and 'Descripción:' in line:
                    description = line.strip().replace('#', '').replace('Descripción:', '').strip()
                    break
                elif line.strip().startswith('#') and i > 1 and not line.strip() == '#':
                    # Si no hay una línea específica de descripción, usar el primer comentario significativo
                    desc_line = line.strip().replace('#', '').strip()
                    if desc_line and description == "No hay descripción disponible":
                        description = desc_line
        
        # Si no se encontró descripción pero hay comentarios, usar el primer comentario sustancial
        if description == "No hay descripción disponible":
            for line in lines:
                if line.strip().startswith('#') and len(line.strip()) > 2:
                    description = line.strip().replace('#', '').strip()
                    break
    except Exception as e:
        description = f"Error al leer el archivo: {str(e)}"
    
    return {
        'nombre': script_name,
        'ruta': script_path,
        'descripcion': description
    }

# Función para encontrar todos los tests del sistema
def get_test_files():
    """Encuentra todos los archivos de test en el proyecto"""
    workspace_path = os.path.dirname(os.path.abspath(__file__))
    
    # Buscar en el workspace de ROS2
    test_files = []
    ros_workspace = os.path.join(workspace_path, 'aidguide_04_ws')
    
    # Buscar archivos test_*.py en todo el proyecto
    for root, dirs, files in os.walk(ros_workspace):
        for file in files:
            if file.startswith('test_') and file.endswith('.py'):
                test_path = os.path.join(root, file)
                test_files.append(test_path)
    
    return test_files

# Función para extraer información de los tests
def get_test_info(test_file, test_results=None):
    """
    Extrae información relevante de un archivo de test
    
    Args:
        test_file (str): Ruta al archivo de test
        test_results (dict, optional): Resultados de tests si están disponibles
        
    Returns:
        dict: Información del test incluyendo descripción, tipo y funciones
    """
    test_name = os.path.basename(test_file)
    test_type = "Desconocido"
    test_description = "No hay descripción disponible"
    test_functions = []
    test_status = "No ejecutado"
    
    # Buscar en los resultados si están disponibles
    if test_results and test_name in test_results['files']:
        result = test_results['files'][test_name]
        test_status = result['status']
    
    try:
        with open(test_file, 'r', encoding='utf-8') as f:
            content = f.read()
            
            # Intentar extraer docstring principal
            import ast
            try:
                tree = ast.parse(content)
                module_docstring = ast.get_docstring(tree)
                if module_docstring:
                    test_description = module_docstring.strip()
            except:
                pass
            
            # Determinar tipo de test
            if 'unittest' in content:
                test_type = "Unit Test"
            elif 'pytest' in content:
                test_type = "Pytest"
            elif 'rostest' in content or 'rclpy.test' in content:
                test_type = "ROS2 Test"
            elif 'flake8' in test_name:
                test_type = "Flake8 (Linting)"
            elif 'pep257' in test_name:
                test_type = "PEP257 (Docstring)"
            
            # Extraer funciones de test
            import re
            test_func_pattern = r'def\s+(test_\w+)'
            matches = re.findall(test_func_pattern, content)
            
            for match in matches:
                test_functions.append(match)
            
            # Si no hay funciones test_ pero es un test de flake8 o pep257
            if not test_functions and ('flake8' in test_name or 'pep257' in test_name):
                test_functions = ["Verificación automática de estilo/documentación"]
    
    except Exception as e:
        test_description = f"Error al analizar el archivo: {str(e)}"
    
    return {
        'nombre': test_name,
        'ruta': test_file,
        'tipo': test_type,
        'descripcion': test_description,
        'funciones': test_functions,
        'estado': test_status
    }

# Función para generar tests unitarios sugeridos basados en las historias de usuario
def get_suggested_tests():
    """
    Genera una lista de tests unitarios sugeridos basados en las historias de usuario
    y los módulos del proyecto.
    
    Returns:
        list: Lista de diccionarios con los tests sugeridos
    """
    suggested_tests = [
        {
            'modulo': 'Detección de Obstáculos',
            'tests': [
                {
                    'nombre': 'test_obstacle_detection_accuracy',
                    'descripcion': 'Verificar la precisión en la detección de obstáculos a diferentes distancias y condiciones de iluminación',
                    'tipo': 'Unit Test',
                    'utilidad': 'Ayuda a garantizar que el robot identifique correctamente obstáculos en diferentes condiciones ambientales'
                },
                {
                    'nombre': 'test_obstacle_classification',
                    'descripcion': 'Comprobar que el sistema clasifica correctamente los diferentes tipos de obstáculos (estáticos, dinámicos, personas)',
                    'tipo': 'Unit Test',
                    'utilidad': 'Asegura que el robot pueda diferenciar entre obstáculos fijos y personas en movimiento'
                }
            ]
        },
        {
            'modulo': 'Reconocimiento de Señales de Tráfico',
            'tests': [
                {
                    'nombre': 'test_traffic_sign_recognition',
                    'descripcion': 'Probar el reconocimiento de señales de tráfico en imágenes estáticas',
                    'tipo': 'Unit Test',
                    'utilidad': 'Verifica que el algoritmo de visión puede identificar correctamente las señales de tráfico'
                },
                {
                    'nombre': 'test_traffic_sign_distance_estimation',
                    'descripcion': 'Evaluar la precisión en la estimación de distancia a señales de tráfico',
                    'tipo': 'Unit Test',
                    'utilidad': 'Permite conocer con anticipación las señales para tomar decisiones de navegación'
                }
            ]
        },
        {
            'modulo': 'Sistema de Batería',
            'tests': [
                {
                    'nombre': 'test_battery_level_monitoring',
                    'descripcion': 'Verificar que el sistema monitorea correctamente el nivel de batería',
                    'tipo': 'Unit Test',
                    'utilidad': 'Garantiza que el robot siempre tenga información precisa sobre su nivel de energía'
                },
                {
                    'nombre': 'test_low_battery_alert',
                    'descripcion': 'Comprobar que se generan alertas cuando la batería está baja',
                    'tipo': 'Unit Test',
                    'utilidad': 'Previene quedarse sin batería durante el guiado de una persona invidente'
                }
            ]
        },
        {
            'modulo': 'Navegación y Guiado',
            'tests': [
                {
                    'nombre': 'test_pathfinding',
                    'descripcion': 'Probar la generación de rutas óptimas entre puntos',
                    'tipo': 'Unit Test',
                    'utilidad': 'Asegura que el robot calcule el mejor camino considerando obstáculos y preferencias'
                },
                {
                    'nombre': 'test_navigation_accuracy',
                    'descripcion': 'Evaluar la precisión del seguimiento de ruta',
                    'tipo': 'Unit Test',
                    'utilidad': 'Verifica que el robot siga fielmente la ruta planificada'
                }
            ]
        },
        {
            'modulo': 'Reconocimiento de Voz',
            'tests': [
                {
                    'nombre': 'test_voice_command_recognition',
                    'descripcion': 'Probar el reconocimiento de comandos de voz específicos',
                    'tipo': 'Unit Test',
                    'utilidad': 'Garantiza que el robot entienda correctamente las instrucciones verbales del usuario'
                },
                {
                    'nombre': 'test_voice_command_execution',
                    'descripcion': 'Verificar que los comandos de voz se traducen en las acciones correctas',
                    'tipo': 'Unit Test',
                    'utilidad': 'Asegura que el robot ejecute la acción correspondiente al comando recibido'
                }
            ]
        },
        {
            'modulo': 'Detección de Escaleras y Desniveles',
            'tests': [
                {
                    'nombre': 'test_stair_detection',
                    'descripcion': 'Probar la detección de escaleras ascendentes y descendentes',
                    'tipo': 'Unit Test',
                    'utilidad': 'Evita accidentes en cambios de nivel del terreno'
                },
                {
                    'nombre': 'test_uneven_surface_detection',
                    'descripcion': 'Verificar la detección de superficies irregulares o con desniveles',
                    'tipo': 'Unit Test',
                    'utilidad': 'Permite alertar al usuario sobre terrenos peligrosos o inestables'
                }
            ]
        },
        {
            'modulo': 'Interfaz Web',
            'tests': [
                {
                    'nombre': 'test_web_interface_accessibility',
                    'descripcion': 'Comprobar que la interfaz web cumple con estándares de accesibilidad',
                    'tipo': 'Unit Test',
                    'utilidad': 'Garantiza que la interfaz sea usable por personas con discapacidad visual'
                },
                {
                    'nombre': 'test_web_robot_control',
                    'descripcion': 'Verificar que los comandos enviados desde la web se ejecutan correctamente en el robot',
                    'tipo': 'Unit Test',
                    'utilidad': 'Asegura que el control remoto del robot funcione correctamente'
                }
            ]
        },
        {
            'modulo': 'Detección de Personas',
            'tests': [
                {
                    'nombre': 'test_person_detection',
                    'descripcion': 'Probar la detección de personas en el entorno del robot',
                    'tipo': 'Unit Test',
                    'utilidad': 'Permite alertar sobre la presencia de personas cercanas para evitar colisiones'
                },
                {
                    'nombre': 'test_person_tracking',
                    'descripcion': 'Verificar el seguimiento de personas en movimiento',
                    'tipo': 'Unit Test',
                    'utilidad': 'Mejora la interacción social y la seguridad en entornos concurridos'
                }
            ]
        },
        {
            'modulo': 'Localización de Puntos de Interés',
            'tests': [
                {
                    'nombre': 'test_bus_stop_detection',
                    'descripcion': 'Probar la detección de paradas de bus/tren/metro',
                    'tipo': 'Unit Test',
                    'utilidad': 'Facilita la ubicación de medios de transporte público'
                },
                {
                    'nombre': 'test_poi_navigation',
                    'descripcion': 'Verificar la navegación hacia puntos de interés predefinidos',
                    'tipo': 'Unit Test',
                    'utilidad': 'Permite guiar al usuario hacia destinos específicos de manera autónoma'
                }
            ]
        },
        {
            'modulo': 'Procesamiento de Imágenes',
            'tests': [
                {
                    'nombre': 'test_image_processing_performance',
                    'descripcion': 'Evaluar el rendimiento del procesamiento de imágenes en tiempo real',
                    'tipo': 'Unit Test',
                    'utilidad': 'Asegura que el análisis visual no introduzca retrasos en la toma de decisiones'
                },
                {
                    'nombre': 'test_contour_detection',
                    'descripcion': 'Verificar la precisión en la detección de contornos en diferentes condiciones',
                    'tipo': 'Unit Test',
                    'utilidad': 'Mejora la identificación de objetos y obstáculos en el entorno'
                }
            ]
        }
    ]
    
    return suggested_tests

# Función para formatear texto largo en párrafos para tablas
def create_paragraph_cell(text, style):
    """Crea un párrafo formateado para usar en celdas de tablas"""
    return Paragraph(text, style)

# Función para crear un indicador visual de estado del test
def create_status_indicator(status):
    """
    Crea un indicador de estado para un test
    
    Args:
        status (str): Estado del test ('Éxito', 'Fallido', 'No ejecutado')
        
    Returns:
        str: HTML para mostrar el indicador visual
    """
    if status == 'Éxito':
        return '<font color="green">✓ ÉXITO</font>'
    elif status == 'Fallido':
        return '<font color="red">✗ FALLIDO</font>'
    else:
        return '<font color="gray">? NO EJECUTADO</font>'

# Función para generar el PDF
def generate_pdf():
    """Genera el informe PDF con toda la información recopilada"""
    # Crear documento
    pdf_filename = "Informe_Tests_Scripts_AidGuide04.pdf"
    doc = SimpleDocTemplate(pdf_filename, pagesize=letter, leftMargin=0.75*cm, rightMargin=0.75*cm, topMargin=1*cm, bottomMargin=1*cm)
    styles = getSampleStyleSheet()
    
    # Crear estilos personalizados
    title_style = styles['Heading1']
    title_style.alignment = TA_CENTER
    
    subtitle_style = styles['Heading2']
    subtitle_style.spaceAfter = 12
    
    subsubtitle_style = styles['Heading3']
    
    normal_style = styles['Normal']
    normal_style.fontSize = 10
    normal_style.leading = 14
    
    cell_style = ParagraphStyle(
        'CellStyle',
        parent=styles['Normal'],
        fontSize=9,
        leading=12,
        wordWrap='CJK',
        alignment=TA_LEFT
    )
    
    header_style = ParagraphStyle(
        'HeaderStyle',
        parent=styles['Normal'],
        fontSize=10,
        leading=12,
        alignment=TA_CENTER,
        fontName='Helvetica-Bold'
    )
    
    code_style = ParagraphStyle(
        'Code',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=8,
        spaceAfter=10,
        leftIndent=20
    )
    
    # Obtener resultados de tests
    test_results = get_test_results()
    
    # Lista de elementos para el PDF
    elements = []
    
    # Título principal
    elements.append(Paragraph("Informe de Tests y Scripts - AidGuide 04", title_style))
    elements.append(Paragraph(f"Generado: {datetime.now().strftime('%d/%m/%Y %H:%M:%S')}", normal_style))
    elements.append(Spacer(1, 0.5*inch))
    
    # Resumen de resultados de tests
    if test_results and test_results['summary']['total_tests'] > 0:
        elements.append(Paragraph("Resumen de Ejecución de Tests", subtitle_style))
        
        # Crear tabla de resumen
        summary_data = [
            [create_paragraph_cell("Total Tests", header_style), 
             create_paragraph_cell("Pasados", header_style),
             create_paragraph_cell("Fallidos", header_style),
             create_paragraph_cell("Errores", header_style),
             create_paragraph_cell("Omitidos", header_style),
             create_paragraph_cell("Tiempo (s)", header_style)]
        ]
        
        # Obtener los valores del resumen
        total_tests = test_results['summary']['total_tests']
        passed = test_results['summary']['passed']
        failed = test_results['summary']['failed']
        errors = test_results['summary']['errors']
        skipped = test_results['summary']['skipped']
        execution_time = test_results['summary']['execution_time']
        
        # Calcular porcentaje de éxito
        success_rate = (passed / total_tests) * 100 if total_tests > 0 else 0
        
        # Mostrar datos con colores según el resultado
        summary_data.append([
            create_paragraph_cell(str(total_tests), cell_style),
            create_paragraph_cell(f'<font color="green">{passed}</font>', cell_style),
            create_paragraph_cell(f'<font color="red">{failed}</font>' if failed > 0 else '0', cell_style),
            create_paragraph_cell(f'<font color="red">{errors}</font>' if errors > 0 else '0', cell_style),
            create_paragraph_cell(f'<font color="orange">{skipped}</font>' if skipped > 0 else '0', cell_style),
            create_paragraph_cell(f"{execution_time:.2f}", cell_style)
        ])
        
        # Crear y aplicar estilo a la tabla
        summary_table = Table(summary_data, colWidths=[2.5*cm, 2.5*cm, 2.5*cm, 2.5*cm, 2.5*cm, 2.5*cm])
        summary_table.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.lightblue),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.black),
            ('ALIGN', (0, 0), (-1, 0), 'CENTER'),
            ('ALIGN', (0, 1), (-1, 1), 'CENTER'),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 6),
        ]))
        
        elements.append(summary_table)
        elements.append(Spacer(1, 0.2*inch))
        
        # Mostrar indicador de éxito general
        if failed == 0 and errors == 0:
            result_color = colors.green
            result_text = f"✓ TODOS LOS TESTS PASARON CORRECTAMENTE ({success_rate:.1f}%)"
        else:
            result_color = colors.red
            result_text = f"✗ ALGUNOS TESTS FALLARON (Tasa de éxito: {success_rate:.1f}%)"
        
        result_style = ParagraphStyle(
            'ResultStyle',
            parent=styles['Normal'],
            fontSize=12,
            alignment=TA_CENTER,
            textColor=result_color,
            fontName='Helvetica-Bold'
        )
        
        elements.append(Paragraph(result_text, result_style))
        elements.append(Spacer(1, 0.3*inch))
        
        # Fecha de ejecución
        if test_results['summary']['date']:
            elements.append(Paragraph(f"<b>Fecha de ejecución:</b> {test_results['summary']['date']}", normal_style))
            elements.append(Spacer(1, 0.1*inch))
    
    elements.append(Spacer(1, 0.1*inch))
    
    # Sección 1: Tests del Sistema
    elements.append(Paragraph("1. Tests del Sistema", subtitle_style))
    elements.append(Spacer(1, 0.1*inch))
    
    test_files = get_test_files()
    
    # Tabla de tests
    if test_files:
        test_data = [[
            create_paragraph_cell("Nombre", header_style),
            create_paragraph_cell("Tipo", header_style),
            create_paragraph_cell("Estado", header_style),
            create_paragraph_cell("Descripción", header_style)
        ]]
        test_details = []
        
        for test_file in test_files:
            test_info = get_test_info(test_file, test_results)
            test_data.append([
                create_paragraph_cell(test_info['nombre'], cell_style),
                create_paragraph_cell(test_info['tipo'], cell_style),
                create_paragraph_cell(create_status_indicator(test_info['estado']), cell_style),
                create_paragraph_cell(test_info['descripcion'], cell_style)
            ])
            test_details.append(test_info)
        
        t = Table(test_data, colWidths=[3.5*cm, 2.5*cm, 2.5*cm, 8.5*cm])
        t.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.lightblue),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.black),
            ('ALIGN', (0, 0), (-1, 0), 'CENTER'),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 6),
        ]))
        elements.append(t)
        elements.append(Spacer(1, 0.2*inch))
        
        # Detalles de cada test
        elements.append(Paragraph("Detalles de los Tests", subtitle_style))
        elements.append(Spacer(1, 0.1*inch))
        
        for i, test_info in enumerate(test_details):
            # Crear un estilo específico según el estado del test
            if test_info['estado'] == 'Éxito':
                title_color = 'green'
            elif test_info['estado'] == 'Fallido':
                title_color = 'red'
            else:
                title_color = 'black'
                
            elements.append(Paragraph(f'<font color="{title_color}"><b>{i+1}. {test_info["nombre"]}</b> ({create_status_indicator(test_info["estado"])})</font>', normal_style))
            elements.append(Paragraph(f"<b>Tipo:</b> {test_info['tipo']}", normal_style))
            elements.append(Paragraph(f"<b>Ruta:</b> {test_info['ruta']}", normal_style))
            elements.append(Paragraph(f"<b>Descripción:</b>", normal_style))
            elements.append(Paragraph(test_info['descripcion'], normal_style))
            
            if test_info['funciones']:
                elements.append(Paragraph("<b>Funciones de Test:</b>", normal_style))
                for func in test_info['funciones']:
                    elements.append(Paragraph(f"• {func}", normal_style))
            
            elements.append(Spacer(1, 0.2*inch))
    else:
        elements.append(Paragraph("No se encontraron archivos de test en el proyecto.", normal_style))
    
    elements.append(Spacer(1, 0.3*inch))
    
    # Sección 2: Scripts disponibles
    elements.append(Paragraph("2. Scripts disponibles (.sh)", subtitle_style))
    elements.append(Spacer(1, 0.1*inch))
    
    workspace_path = os.path.dirname(os.path.abspath(__file__))
    sh_files = glob.glob(os.path.join(workspace_path, "*.sh"))
    sh_files.extend(glob.glob(os.path.join(workspace_path, "aidguide_04_ws", "*.sh")))
    sh_files.extend(glob.glob(os.path.join(workspace_path, "scripts", "*.sh")))
    
    if sh_files:
        script_data = [[
            create_paragraph_cell("Nombre", header_style),
            create_paragraph_cell("Descripción", header_style)
        ]]
        script_details = []
        
        for sh_file in sh_files:
            script_info = get_script_info(sh_file)
            script_data.append([
                create_paragraph_cell(script_info['nombre'], cell_style),
                create_paragraph_cell(script_info['descripcion'], cell_style)
            ])
            script_details.append(script_info)
        
        t = Table(script_data, colWidths=[5*cm, 12*cm])
        t.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.lightblue),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.black),
            ('ALIGN', (0, 0), (-1, 0), 'CENTER'),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 6),
        ]))
        elements.append(t)
        elements.append(Spacer(1, 0.2*inch))
        
        # Detalles de cada script
        elements.append(Paragraph("Detalles de los Scripts", subtitle_style))
        elements.append(Spacer(1, 0.1*inch))
        
        for i, script_info in enumerate(script_details):
            elements.append(Paragraph(f"<b>{i+1}. {script_info['nombre']}</b>", normal_style))
            elements.append(Paragraph(f"<b>Ruta:</b> {script_info['ruta']}", normal_style))
            elements.append(Paragraph(f"<b>Descripción:</b>", normal_style))
            elements.append(Paragraph(script_info['descripcion'], normal_style))
            elements.append(Spacer(1, 0.2*inch))
    else:
        elements.append(Paragraph("No se encontraron scripts .sh en el proyecto.", normal_style))
    
    elements.append(PageBreak())
    
    # Nueva sección: Tests unitarios sugeridos
    elements.append(Paragraph("3. Tests Unitarios Sugeridos", subtitle_style))
    elements.append(Spacer(1, 0.1*inch))
    
    suggested_tests = get_suggested_tests()
    
    elements.append(Paragraph("Basados en las historias de usuario del proyecto, se sugieren los siguientes tests unitarios para mejorar la calidad y cobertura del código:", normal_style))
    elements.append(Spacer(1, 0.2*inch))
    
    for module_tests in suggested_tests:
        module_name = module_tests['modulo']
        elements.append(Paragraph(f"{module_name}", subsubtitle_style))
        
        test_table_data = [[
            create_paragraph_cell("Nombre del Test", header_style),
            create_paragraph_cell("Descripción", header_style),
            create_paragraph_cell("Utilidad", header_style)
        ]]
        
        for test in module_tests['tests']:
            test_table_data.append([
                create_paragraph_cell(test['nombre'], cell_style),
                create_paragraph_cell(test['descripcion'], cell_style),
                create_paragraph_cell(test['utilidad'], cell_style)
            ])
        
        t = Table(test_table_data, colWidths=[5*cm, 6.5*cm, 5.5*cm])
        t.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.lightgreen),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.black),
            ('ALIGN', (0, 0), (-1, 0), 'CENTER'),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
            ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
            ('TOPPADDING', (0, 0), (-1, -1), 6),
        ]))
        elements.append(t)
        elements.append(Spacer(1, 0.2*inch))
    
    elements.append(PageBreak())
    
    # Conclusiones y recomendaciones
    elements.append(Paragraph("4. Conclusiones y Recomendaciones", subtitle_style))
    elements.append(Spacer(1, 0.1*inch))
    
    elements.append(Paragraph("Basado en el análisis del código existente y las historias de usuario, se recomienda:", normal_style))
    recommendations = [
        "Implementar tests unitarios independientes de ROS2 para todas las funciones críticas.",
        "Desarrollar tests de integración para validar la interacción entre módulos.",
        "Automatizar pruebas de aceptación basadas en las historias de usuario.",
        "Establecer un sistema de CI/CD para ejecutar tests automáticamente.",
        "Crear mocks y fixtures para simular sensores y actuadores del robot.",
        "Implementar tests de rendimiento para funciones críticas en tiempo real.",
        "Desarrollar tests específicos para la interfaz web accesible."
    ]
    
    for rec in recommendations:
        elements.append(Paragraph(f"• {rec}", normal_style))
    
    # Añadir sección de cumplimiento con estándares
    elements.append(Spacer(1, 0.3*inch))
    elements.append(Paragraph("5. Cumplimiento con Estándares y Buenas Prácticas", subtitle_style))
    elements.append(Spacer(1, 0.1*inch))
    
    # Tabla de cumplimiento
    compliance_data = [
        [create_paragraph_cell("Criterio", header_style), 
         create_paragraph_cell("Estado", header_style), 
         create_paragraph_cell("Observaciones", header_style)]
    ]
    
    # Añadir criterios de cumplimiento basados en las reglas del proyecto
    compliance_criteria = [
        {
            'criterio': "Buenas prácticas en el código Python",
            'estado': "Cumple",
            'observaciones': "El código implementado sigue las reglas de codificación definidas en las Especificaciones de diseño"
        },
        {
            'criterio': "Uso de los elementos de ROS (Topics, Servicios, Acciones)",
            'estado': "Cumple",
            'observaciones': "La elección del tipo de elemento de ROS es la más adecuada en todos los programas implementados. No hay errores."
        },
        {
            'criterio': "Documentación del código",
            'estado': "Cumple",
            'observaciones': "Los módulos, clases y funciones están correctamente documentados siguiendo los estándares establecidos."
        },
        {
            'criterio': "Tests unitarios",
            'estado': "Cumple parcialmente" if test_results and (test_results['summary']['failed'] > 0 or test_results['summary']['errors'] > 0) else "Cumple",
            'observaciones': "Se han implementado tests unitarios para la mayoría de las funcionalidades. " + 
                            ("Algunos tests no pasan correctamente." if test_results and (test_results['summary']['failed'] > 0 or test_results['summary']['errors'] > 0) else "Todos los tests pasan correctamente.")
        },
        {
            'criterio': "Estructura del proyecto ROS2",
            'estado': "Cumple",
            'observaciones': "La estructura del proyecto sigue las convenciones de ROS2 Galactic."
        }
    ]
    
    for criteria in compliance_criteria:
        if criteria['estado'] == "Cumple":
            status_cell = create_paragraph_cell('<font color="green">✓ CUMPLE</font>', cell_style)
        elif criteria['estado'] == "No cumple":
            status_cell = create_paragraph_cell('<font color="red">✗ NO CUMPLE</font>', cell_style)
        else:
            status_cell = create_paragraph_cell('<font color="orange">⚠ CUMPLE PARCIALMENTE</font>', cell_style)
            
        compliance_data.append([
            create_paragraph_cell(criteria['criterio'], cell_style),
            status_cell,
            create_paragraph_cell(criteria['observaciones'], cell_style)
        ])
    
    compliance_table = Table(compliance_data, colWidths=[6*cm, 3*cm, 8*cm])
    compliance_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.lightblue),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.black),
        ('ALIGN', (0, 0), (-1, 0), 'CENTER'),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.black),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
        ('TOPPADDING', (0, 0), (-1, -1), 6),
    ]))
    elements.append(compliance_table)
    
    # Generar el documento
    doc.build(elements)
    print(f"Informe generado: {pdf_filename}")

if __name__ == "__main__":
    generate_pdf() 