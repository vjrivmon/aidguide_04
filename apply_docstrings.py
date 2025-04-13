#!/usr/bin/env python3
"""Script para aplicar automáticamente docstrings a archivos Python.

Este script analiza archivos Python del proyecto y agrega docstrings
faltantes según los estándares definidos.

Usage:
    python3 apply_docstrings.py [archivo_o_directorio]
"""

import sys
import os
import re
import ast
from typing import List, Dict, Set, Tuple, Optional

def extract_info_from_ast(node, file_content: str) -> Dict:
    """Extrae información necesaria de un nodo AST.
    
    Args:
        node: Nodo AST a analizar
        file_content: Contenido del archivo fuente
        
    Returns:
        Dict: Información extraída del nodo
    """
    info = {
        'docstring': ast.get_docstring(node),
        'lineno': getattr(node, 'lineno', 1),
        'name': getattr(node, 'name', '<module>'),
        'params': [],
        'returns': False,
        'raises': set(),
        'attributes': []
    }
    
    # Extraer información específica según el tipo de nodo
    if isinstance(node, ast.FunctionDef):
        # Extraer parámetros
        for arg in node.args.args:
            if arg.arg != 'self':
                arg_type = 'Any'
                if arg.annotation:
                    if isinstance(arg.annotation, ast.Name):
                        arg_type = arg.annotation.id
                    elif isinstance(arg.annotation, ast.Attribute):
                        arg_type = arg.annotation.attr
                info['params'].append((arg.arg, arg_type))
        
        # Detectar retornos
        for child in ast.walk(node):
            if isinstance(child, ast.Return) and child.value is not None:
                info['returns'] = True
            
            # Detectar excepciones
            if isinstance(child, ast.Raise):
                if isinstance(child.exc, ast.Name):
                    info['raises'].add(child.exc.id)
                elif isinstance(child.exc, ast.Call) and isinstance(child.exc.func, ast.Name):
                    info['raises'].add(child.exc.func.id)
    
    elif isinstance(node, ast.ClassDef):
        # Extraer atributos
        for child in node.body:
            if isinstance(child, ast.Assign):
                for target in child.targets:
                    if isinstance(target, ast.Name):
                        info['attributes'].append(target.id)
            elif isinstance(child, ast.AnnAssign) and isinstance(child.target, ast.Name):
                info['attributes'].append(child.target.id)
    
    return info

def generate_docstring(node_type: str, info: Dict) -> str:
    """Genera un docstring para un nodo según su tipo.
    
    Args:
        node_type: Tipo de nodo ('module', 'class', 'function')
        info: Información del nodo
        
    Returns:
        str: Docstring generado
    """
    if node_type == 'module':
        module_name = info['name']
        return f'"""Módulo {module_name}.\n\nEste módulo proporciona funcionalidades para el proyecto AidGuide 04.\n"""'
    
    elif node_type == 'class':
        class_name = info['name']
        docstring = f'"""Clase {class_name}.\n\nImplementa funcionalidad para {class_name}.'
        
        if info['attributes']:
            docstring += '\n\nAttributes:'
            for attr in info['attributes']:
                docstring += f'\n    {attr}: Descripción del atributo.'
        
        docstring += '\n"""'
        return docstring
    
    elif node_type == 'function':
        func_name = info['name']
        
        # Mejor nombre para display
        display_name = func_name.replace('_', ' ').strip()
        if func_name == '__init__':
            display_name = 'Constructor'
        
        docstring = f'"""Función {display_name.capitalize()}.'
        
        if info['params']:
            docstring += '\n\nArgs:'
            for param_name, param_type in info['params']:
                docstring += f'\n    {param_name} ({param_type}): Descripción del parámetro.'
        
        if info['returns']:
            docstring += '\n\nReturns:\n    Descripción del valor de retorno.'
        
        if info['raises']:
            docstring += '\n\nRaises:'
            for exc in info['raises']:
                docstring += f'\n    {exc}: Descripción de cuándo se lanza esta excepción.'
        
        docstring += '\n"""'
        return docstring
    
    return '"""Docstring por defecto.\n"""'

def fix_file_docstrings(file_path: str) -> List[Tuple[int, str, str]]:
    """Corrige docstrings en un archivo Python.
    
    Args:
        file_path: Ruta al archivo a corregir
    
    Returns:
        List[Tuple[int, str, str]]: Lista de cambios (línea, tipo, descripción)
    """
    changes = []
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Parsear el código
        tree = ast.parse(content)
        
        # Lista de modificaciones a realizar (línea, docstring, indentación)
        modifications = []
        
        # Verificar módulo
        module_info = extract_info_from_ast(tree, content)
        if not module_info['docstring']:
            module_docstring = generate_docstring('module', module_info)
            # Buscar la línea apropiada para insertar
            import_lines = []
            for i, node in enumerate(tree.body):
                if isinstance(node, ast.Import) or isinstance(node, ast.ImportFrom):
                    import_lines.append(node.lineno)
            
            # Insertar después de la última importación o al principio
            insert_line = max(import_lines) if import_lines else 1
            modifications.append((insert_line, module_docstring, ""))
            changes.append((1, 'module', f"Añadido docstring para módulo"))
        
        # Verificar clases y funciones
        for node in ast.walk(tree):
            if isinstance(node, (ast.ClassDef, ast.FunctionDef)):
                # Ignorar métodos privados que no sean dunder methods
                if isinstance(node, ast.FunctionDef) and node.name.startswith('_') and not (node.name.startswith('__') and node.name.endswith('__')):
                    continue
                
                node_info = extract_info_from_ast(node, content)
                
                if not node_info['docstring']:
                    node_type = 'class' if isinstance(node, ast.ClassDef) else 'function'
                    docstring = generate_docstring(node_type, node_info)
                    
                    # Determinar indentación
                    lines = content.splitlines()
                    line_idx = node.lineno - 1
                    if line_idx < len(lines):
                        indent_match = re.match(r'^(\s*)', lines[line_idx])
                        indent = indent_match.group(1) if indent_match else ""
                        # Añadir indentación adicional para el cuerpo
                        body_indent = indent + "    "
                    else:
                        indent = ""
                        body_indent = "    "
                    
                    # Modificar docstring para ajustar indentación
                    docstring_lines = docstring.splitlines()
                    indented_docstring = docstring_lines[0] + '\n'
                    for line in docstring_lines[1:]:
                        indented_docstring += body_indent + line + '\n'
                    indented_docstring = indented_docstring.rstrip()
                    
                    # Encontrar la línea para insertar (primera del cuerpo)
                    # Si hay nodos en el cuerpo, usar la línea del primero
                    if hasattr(node, 'body') and node.body:
                        insert_line = node.body[0].lineno
                    else:
                        insert_line = node.lineno + 1
                    
                    modifications.append((insert_line, indented_docstring, body_indent))
                    changes.append((node.lineno, node_type, f"Añadido docstring para {node_type} {node.name}"))
        
        # Aplicar modificaciones si hay cambios
        if modifications:
            # Ordenar modificaciones de abajo hacia arriba para evitar desfases de líneas
            modifications.sort(reverse=True, key=lambda x: x[0])
            
            lines = content.splitlines()
            
            # Aplicar cada modificación
            for line_num, docstring, indent in modifications:
                # Ajustar índice (líneas empiezan en 1, lista en 0)
                idx = min(line_num - 1, len(lines))
                lines.insert(idx, indent + docstring)
            
            # Guardar cambios
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines))
    
    except Exception as e:
        print(f"Error procesando {file_path}: {str(e)}")
        changes.append((0, 'error', f"Error: {str(e)}"))
    
    return changes

def fix_directory(directory: str, priority_patterns: List[str] = None) -> Dict[str, List[Tuple]]:
    """Corrige docstrings en todos los archivos Python de un directorio.
    
    Args:
        directory: Ruta al directorio a corregir
        priority_patterns: Lista de patrones para priorizar archivos
        
    Returns:
        Dict[str, List[Tuple]]: Diccionario con archivos y cambios realizados
    """
    if priority_patterns is None:
        priority_patterns = ["nav", "monitor", "robot", "dashboard", "interface", "web"]
    
    results = {}
    priority_files = []
    regular_files = []
    
    # Primero identificar todos los archivos
    for root, _, files in os.walk(directory):
        for file in sorted(files):
            if file.endswith('.py'):
                file_path = os.path.join(root, file)
                is_priority = any(pattern in file_path.lower() for pattern in priority_patterns)
                
                if is_priority:
                    priority_files.append(file_path)
                else:
                    regular_files.append(file_path)
    
    # Procesar archivos prioritarios primero
    for file_path in priority_files:
        changes = fix_file_docstrings(file_path)
        if changes:
            results[file_path] = changes
            print(f"✅ Corregido (prioritario): {file_path} - {len(changes)} cambios")
    
    # Luego procesar archivos regulares
    for file_path in regular_files:
        changes = fix_file_docstrings(file_path)
        if changes:
            results[file_path] = changes
            print(f"✅ Corregido: {file_path} - {len(changes)} cambios")
    
    return results

def main():
    """Función principal."""
    if len(sys.argv) < 2:
        print("Uso: python3 apply_docstrings.py [archivo_o_directorio]")
        return
    
    target = sys.argv[1]
    if not os.path.exists(target):
        print(f"Error: El archivo o directorio {target} no existe")
        return
    
    print(f"Aplicando docstrings a {target}...\n")
    
    if os.path.isfile(target):
        changes = fix_file_docstrings(target)
        if changes:
            print(f"\nSe realizaron {len(changes)} cambios en {target}:")
            for line, node_type, description in changes:
                print(f"  • Línea {line} ({node_type}): {description}")
        else:
            print(f"\nNo se realizaron cambios en {target}")
    else:
        priority_patterns = ["nav", "monitor", "robot", "dashboard", "interface", "web"]
        results = fix_directory(target, priority_patterns)
        
        total_files = len(results)
        total_changes = sum(len(changes) for changes in results.values())
        
        if total_changes == 0:
            print("\n✅ No se requirieron cambios de docstrings.")
            return
        
        print(f"\n✅ Se realizaron {total_changes} cambios en {total_files} archivos:\n")
        
        for file_path, changes in sorted(results.items()):
            print(f"\n📁 {file_path} ({len(changes)} cambios):")
            for line, node_type, description in changes:
                print(f"  • Línea {line} ({node_type}): {description}")
        
        print(f"\nResumen:")
        print(f"- Total archivos corregidos: {total_files}")
        print(f"- Total cambios realizados: {total_changes}")

if __name__ == "__main__":
    main() 