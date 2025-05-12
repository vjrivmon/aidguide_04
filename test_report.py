#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Generador de Informes de Tests y Scripts para AidGuide 04
Este script genera un informe PDF de los tests y scripts disponibles en el proyecto.
"""

import os
import glob
import subprocess
from datetime import datetime
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, ListFlowable, ListItem
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch

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
def get_test_info(test_file):
    """Extrae información relevante de un archivo de test"""
    test_name = os.path.basename(test_file)
    test_type = "Desconocido"
    test_description = "No hay descripción disponible"
    test_functions = []
    
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
        'funciones': test_functions
    }

# Función para generar el PDF
def generate_pdf():
    """Genera el informe PDF con toda la información recopilada"""
    # Crear documento
    pdf_filename = "Informe_Tests_Scripts_AidGuide04.pdf"
    doc = SimpleDocTemplate(pdf_filename, pagesize=letter)
    styles = getSampleStyleSheet()
    
    # Crear estilos personalizados
    title_style = styles['Heading1']
    subtitle_style = styles['Heading2']
    normal_style = styles['Normal']
    code_style = ParagraphStyle(
        'Code',
        parent=styles['Normal'],
        fontName='Courier',
        fontSize=8,
        spaceAfter=10,
        leftIndent=20
    )
    
    # Lista de elementos para el PDF
    elements = []
    
    # Título principal
    elements.append(Paragraph("Informe de Tests y Scripts - AidGuide 04", title_style))
    elements.append(Paragraph(f"Generado: {datetime.now().strftime('%d/%m/%Y %H:%M:%S')}", normal_style))
    elements.append(Spacer(1, 0.5*inch))
    
    # Sección 1: Tests del Sistema
    elements.append(Paragraph("1. Tests del Sistema", subtitle_style))
    elements.append(Spacer(1, 0.1*inch))
    
    test_files = get_test_files()
    
    # Tabla de tests
    if test_files:
        test_data = [["Nombre", "Tipo", "Descripción"]]
        test_details = []
        
        for test_file in test_files:
            test_info = get_test_info(test_file)
            test_data.append([
                test_info['nombre'], 
                test_info['tipo'], 
                test_info['descripcion'][:100] + "..." if len(test_info['descripcion']) > 100 else test_info['descripcion']
            ])
            test_details.append(test_info)
        
        t = Table(test_data, colWidths=[1.5*inch, 1*inch, 3.5*inch])
        t.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.lightblue),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.black),
            ('ALIGN', (0, 0), (-1, 0), 'CENTER'),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
            ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ]))
        elements.append(t)
        elements.append(Spacer(1, 0.2*inch))
        
        # Detalles de cada test
        elements.append(Paragraph("Detalles de los Tests", subtitle_style))
        elements.append(Spacer(1, 0.1*inch))
        
        for i, test_info in enumerate(test_details):
            elements.append(Paragraph(f"<b>{i+1}. {test_info['nombre']}</b>", normal_style))
            elements.append(Paragraph(f"<b>Tipo:</b> {test_info['tipo']}", normal_style))
            elements.append(Paragraph(f"<b>Ruta:</b> {test_info['ruta']}", normal_style))
            elements.append(Paragraph(f"<b>Descripción:</b>", normal_style))
            elements.append(Paragraph(test_info['descripcion'], normal_style))
            
            if test_info['funciones']:
                elements.append(Paragraph("<b>Funciones de Test:</b>", normal_style))
                for func in test_info['funciones']:
                    elements.append(Paragraph(f"- {func}", normal_style))
            
            elements.append(Spacer(1, 0.2*inch))
    else:
        elements.append(Paragraph("No se encontraron archivos de test en el proyecto.", normal_style))
    
    elements.append(Spacer(1, 0.5*inch))
    
    # Sección 2: Scripts disponibles
    elements.append(Paragraph("2. Scripts disponibles (.sh)", subtitle_style))
    elements.append(Spacer(1, 0.1*inch))
    
    workspace_path = os.path.dirname(os.path.abspath(__file__))
    sh_files = glob.glob(os.path.join(workspace_path, "*.sh"))
    sh_files.extend(glob.glob(os.path.join(workspace_path, "aidguide_04_ws", "*.sh")))
    sh_files.extend(glob.glob(os.path.join(workspace_path, "scripts", "*.sh")))
    
    if sh_files:
        script_data = [["Nombre", "Descripción"]]
        script_details = []
        
        for sh_file in sh_files:
            script_info = get_script_info(sh_file)
            script_data.append([
                script_info['nombre'],
                script_info['descripcion'][:100] + "..." if len(script_info['descripcion']) > 100 else script_info['descripcion']
            ])
            script_details.append(script_info)
        
        t = Table(script_data, colWidths=[2*inch, 4*inch])
        t.setStyle(TableStyle([
            ('BACKGROUND', (0, 0), (-1, 0), colors.lightblue),
            ('TEXTCOLOR', (0, 0), (-1, 0), colors.black),
            ('ALIGN', (0, 0), (-1, 0), 'CENTER'),
            ('GRID', (0, 0), (-1, -1), 0.5, colors.black),
            ('VALIGN', (0, 0), (-1, -1), 'TOP'),
            ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
        ]))
        elements.append(t)
        elements.append(Spacer(1, 0.2*inch))
        
        # Detalles de cada script
        elements.append(Paragraph("Detalles de los Scripts", subtitle_style))
        elements.append(Spacer(1, 0.1*inch))
        
        for i, script_info in enumerate(script_details):
            elements.append(Paragraph(f"<b>{i+1}. {script_info['nombre']}</b>", normal_style))
            elements.append(Paragraph(f"<b>Ruta:</b> {script_info['ruta']}", normal_style))
            elements.append(Paragraph(f"<b>Descripción:</b> {script_info['descripcion']}", normal_style))
            elements.append(Spacer(1, 0.2*inch))
    else:
        elements.append(Paragraph("No se encontraron scripts .sh en el proyecto.", normal_style))
    
    # Crear el PDF
    doc.build(elements)
    print(f"Informe PDF generado: {pdf_filename}")
    return pdf_filename

if __name__ == "__main__":
    pdf_file = generate_pdf()
    print(f"Informe creado exitosamente: {pdf_file}")
    # Intentar abrir automáticamente el PDF
    try:
        if os.name == 'nt':  # Windows
            os.startfile(pdf_file)
        elif os.name == 'posix':  # Linux/Mac
            subprocess.call(['xdg-open', pdf_file])
    except:
        print("No se pudo abrir el archivo automáticamente. Por favor, ábralo manualmente.") 