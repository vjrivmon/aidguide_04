from flask import Flask, request, send_file, jsonify
import cv2
import numpy as np
import os
import io
from flask_cors import CORS

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PUBLIC_DIR = os.path.join(BASE_DIR, "aidguide_04_web", "public")

app = Flask(__name__)
CORS(app)

def detectar_colores(img_path):
    img = cv2.imread(img_path)
    if img is None:
        return None

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Azul (ajustado para señales azules)
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([130, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # Rojo (ajustado para señales de stop y prohibición)
    lower_red1 = np.array([0, 150, 50])
    upper_red1 = np.array([10, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    lower_red2 = np.array([170, 150, 50])
    upper_red2 = np.array([180, 255, 255])
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Amarillo (ajustado para señales de advertencia)
    lower_yellow = np.array([20, 150, 100])
    upper_yellow = np.array([35, 255, 255])
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Combinar máscaras
    mask = cv2.bitwise_or(mask_blue, mask_red)
    mask = cv2.bitwise_or(mask, mask_yellow)

    # Aplicar la máscara a la imagen original
    res = cv2.bitwise_and(img, img, mask=mask)
    
    # Mejorar la visualización
    # Convertir a escala de grises para el fondo
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    
    # Combinar la imagen original con los colores detectados
    alpha = 0.7
    res = cv2.addWeighted(res, alpha, gray, 1-alpha, 0)
    
    return res

def aplicar_canny(img_path):
    """
    Aplica el algoritmo de Canny para detectar bordes en una imagen.
    
    Args:
        img_path (str): Ruta a la imagen de origen
        
    Returns:
        numpy.ndarray: Imagen con los bordes detectados o None si hay error
    """
    try:
        # Cargar la imagen
        img = cv2.imread(img_path)
        if img is None:
            return None
            
        # Crear una copia para los resultados
        result = img.copy()
        
        # 1. Convertir a escala de grises
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 2. Aplicar suavizado Gaussiano para reducir ruido
        # Usamos un kernel de 5x5 con desviación estándar 0
        img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
        
        # 3. Aplicar algoritmo de Canny para detección de bordes
        # Ajustamos los umbrales para una mejor detección
        edges_canny = cv2.Canny(img_blur, 50, 150)
        
        # 4. Encontrar contornos a partir de los bordes detectados
        contornos, _ = cv2.findContours(
            edges_canny, 
            cv2.RETR_EXTERNAL,  # Solo contornos externos
            cv2.CHAIN_APPROX_SIMPLE  # Comprime segmentos horizontales/verticales
        )
        
        # Crear tres imágenes de resultado para una visualización más completa
        
        # Imagen 1: Bordes en blanco y negro (Canny original)
        edges_bw = cv2.cvtColor(edges_canny, cv2.COLOR_GRAY2BGR)
        
        # Imagen 2: Bordes en color sobre imagen original
        # Dibujar los contornos encontrados en verde sobre la imagen original
        edges_contours = result.copy()
        cv2.drawContours(
            edges_contours, 
            contornos, 
            -1,  # Dibujar todos los contornos
            (0, 255, 0),  # Color verde
            2,  # Grosor de línea
            cv2.LINE_AA  # Tipo de línea suavizada
        )
        
        # Imagen 3: Bordes en naranja sobre imagen original (estilo anterior)
        # Dilatación para mejorar la visibilidad de los bordes
        kernel = np.ones((3, 3), np.uint8)
        edges_dilated = cv2.dilate(edges_canny, kernel, iterations=1)
        
        # Convertir bordes a BGR para poder colorearlos
        edges_color = cv2.cvtColor(edges_dilated, cv2.COLOR_GRAY2BGR)
        
        # Colorear los bordes (naranja)
        edges_color[np.where((edges_color == [255, 255, 255]).all(axis=2))] = [0, 165, 255]
        
        # Combinar con la imagen original para mejor visualización
        edges_overlay = cv2.addWeighted(img, 0.7, edges_color, 0.9, 0)
        
        # Combinar las tres vistas en una sola imagen
        # Primero ajustamos todas las imágenes al mismo tamaño
        height, width = img.shape[:2]
        
        # Crear un lienzo para las tres imágenes
        combined = np.zeros((height, width * 3, 3), dtype=np.uint8)
        
        # Colocar las tres imágenes lado a lado
        combined[:, 0:width] = edges_bw
        combined[:, width:width*2] = edges_contours
        combined[:, width*2:width*3] = edges_overlay
        
        # Agregar textos explicativos
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(combined, "Canny", (10, 30), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(combined, "Contornos", (width + 10, 30), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(combined, "Superpuesto", (width * 2 + 10, 30), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
        
        # Agregar líneas divisorias
        cv2.line(combined, (width, 0), (width, height), (255, 255, 255), 2)
        cv2.line(combined, (width * 2, 0), (width * 2, height), (255, 255, 255), 2)
        
        # Información de contornos
        num_contornos = len(contornos)
        cv2.putText(
            combined, 
            f"Contornos: {num_contornos}", 
            (width + 10, height - 20), 
            font, 
            0.7, 
            (0, 255, 0), 
            2, 
            cv2.LINE_AA
        )
        
        return combined
    except Exception as e:
        print(f"Error al aplicar Canny: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

def detectar_formas(img_path):
    """
    Detecta únicamente señales de tráfico en una imagen.
    
    Args:
        img_path (str): Ruta a la imagen de origen
        
    Returns:
        numpy.ndarray: Imagen con las señales detectadas o None si hay error
    """
    try:
        # Cargar la imagen
        img = cv2.imread(img_path)
        if img is None:
            return None
            
        # Crear una copia para dibujar los resultados
        result = img.copy()
        
        # Convertir a HSV para detección de colores de señales
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Máscaras para colores típicos de señales
        # Rojo (común en señales de prohibición y stop)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        mask_red1 = cv2.inRange(img_hsv, lower_red1, upper_red1)
        
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])
        mask_red2 = cv2.inRange(img_hsv, lower_red2, upper_red2)
        
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # Azul (señales de obligación)
        lower_blue = np.array([100, 80, 50])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(img_hsv, lower_blue, upper_blue)
        
        # Amarillo (señales de advertencia)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
        
        # Combinar máscaras de colores para señales
        mask_colors = cv2.bitwise_or(mask_red, mask_blue)
        mask_colors = cv2.bitwise_or(mask_colors, mask_yellow)
        
        # Aplicar operaciones morfológicas para limpiar la máscara
        kernel = np.ones((5, 5), np.uint8)
        mask_colors = cv2.morphologyEx(mask_colors, cv2.MORPH_CLOSE, kernel)
        mask_colors = cv2.morphologyEx(mask_colors, cv2.MORPH_OPEN, kernel)
        
        # Buscar contornos sólo en la máscara de colores de señales
        contornos, _ = cv2.findContours(
            mask_colors, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Filtrar contornos por área y propiedades
        contornos_filtrados = []
        min_area = 500  # Área mínima más alta para filtrar pequeños objetos
        
        print(f"Contornos preliminares: {len(contornos)}")
        
        for contorno in contornos:
            area = cv2.contourArea(contorno)
            if area < min_area:
                continue
                
            # Comprobar solidez
            x, y, w, h = cv2.boundingRect(contorno)
            area_rectangulo = w * h
            solidez = float(area) / max(area_rectangulo, 1)
            
            # Solo mantener formas con buena solidez
            if solidez < 0.4:
                continue
                
            # Comprobar si es aproximadamente simétrico (característica de señales)
            # Calculamos la relación de aspecto
            relacion_aspecto = float(w) / max(h, 1)
            if relacion_aspecto > 2.0 or relacion_aspecto < 0.5:
                continue
                
            contornos_filtrados.append(contorno)
        
        print(f"Señales detectadas: {len(contornos_filtrados)}")
        
        # Iterar sobre los contornos de señales
        for contorno in contornos_filtrados:
            # Perímetro para aproximación de polígono
            perimetro = cv2.arcLength(contorno, True)
            
            # Aproximar el polígono con un epsilon adecuado para señales
            epsilon = 0.04 * perimetro
            poligono_aproximado = cv2.approxPolyDP(contorno, epsilon, True)
            
            # Propiedades del contorno
            x, y, w, h = cv2.boundingRect(poligono_aproximado)
            area = cv2.contourArea(contorno)
            area_rectangulo = w * h
            solidez = float(area) / max(area_rectangulo, 1)
            relacion_aspecto = float(w) / max(h, 1)
            
            # Obtener círculo mínimo
            (cx, cy), radio = cv2.minEnclosingCircle(contorno)
            area_circulo = np.pi * (radio ** 2)
            circularidad = float(area) / max(area_circulo, 1)
            
            # Número de vértices
            numero_vertices = len(poligono_aproximado)
            
            # Información de diagnóstico
            print(f"Señal: vertices={numero_vertices}, aspecto={relacion_aspecto:.2f}, solidez={solidez:.2f}, circularidad={circularidad:.2f}")
            
            # Clasificación específica de señales
            forma = None
            color_contorno = None
            
            # Clasificar por forma y color
            if 3 <= numero_vertices <= 5 and solidez > 0.5:
                # Triángulos - Señales de advertencia (generalmente amarillas)
                if cv2.countNonZero(cv2.bitwise_and(mask_yellow, cv2.drawContours(np.zeros_like(mask_yellow), [contorno], 0, 255, -1))) > 0:
                    forma = "Senal de Advertencia"
                    color_contorno = (0, 165, 255)  # Naranja
            elif numero_vertices == 4 and 0.9 <= relacion_aspecto <= 1.1 and solidez > 0.7:
                # Cuadrados - Señales informativas (azules)
                if cv2.countNonZero(cv2.bitwise_and(mask_blue, cv2.drawContours(np.zeros_like(mask_blue), [contorno], 0, 255, -1))) > 0:
                    forma = "Senal de Informacion"
                    color_contorno = (255, 0, 0)  # Azul
            elif numero_vertices == 4 and solidez > 0.7:
                # Rectángulos - Señales complementarias
                forma = "Senal Complementaria"
                color_contorno = (0, 255, 0)  # Verde
            elif (8 <= numero_vertices <= 10 and solidez > 0.6) or (circularidad > 0.7 and solidez > 0.6):
                # Circulares - Prohibición (rojas)
                if cv2.countNonZero(cv2.bitwise_and(mask_red, cv2.drawContours(np.zeros_like(mask_red), [contorno], 0, 255, -1))) > 0:
                    forma = "Senal de Prohibicion"
                    color_contorno = (0, 0, 255)  # Rojo
            elif 7 <= numero_vertices <= 9 and solidez > 0.7:
                # Octagonal - STOP
                if cv2.countNonZero(cv2.bitwise_and(mask_red, cv2.drawContours(np.zeros_like(mask_red), [contorno], 0, 255, -1))) > 0:
                    forma = "Senal de STOP"
                    color_contorno = (0, 0, 255)  # Rojo
            
            # Solo procesar formas clasificadas como señales
            if forma and color_contorno:
                # Dibujar el contorno de la señal
                cv2.drawContours(result, [poligono_aproximado], 0, color_contorno, 3)
                
                # Calcular centro para texto
                M = cv2.moments(contorno)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = x + w // 2, y + h // 2
                
                # Colocar etiqueta con fondo semitransparente
                etiqueta = forma
                tamaño_texto = cv2.getTextSize(etiqueta, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                
                overlay = result.copy()
                cv2.rectangle(
                    overlay, 
                    (cx - 5, cy - tamaño_texto[1] - 5), 
                    (cx + tamaño_texto[0] + 5, cy + 5), 
                    color_contorno, 
                    -1
                )
                cv2.addWeighted(overlay, 0.7, result, 0.3, 0, result)
                
                # Colocar texto
                cv2.putText(
                    result, 
                    etiqueta, 
                    (cx, cy), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.7, 
                    (255, 255, 255), 
                    2
                )
        
        return result
    except Exception as e:
        print(f"Error al detectar formas: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

def detectar_blobs(img_path):
    """
    Detecta blobs (objetos binarios grandes) en una imagen y marca sus centroides.
    
    Args:
        img_path (str): Ruta a la imagen de origen
        
    Returns:
        numpy.ndarray: Imagen con los blobs detectados y sus centroides o None si hay error
    """
    try:
        # Cargar la imagen
        img = cv2.imread(img_path)
        if img is None:
            return None
            
        # Crear una copia para dibujar los resultados
        result = img.copy()
        
        # Convertir a escala de grises
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Aplicar suavizado para reducir ruido
        img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
        
        # Aplicar umbralización adaptativa
        thresh = cv2.adaptiveThreshold(
            img_blur,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            11,
            2
        )
        
        # Limpieza de la imagen binaria
        kernel = np.ones((3, 3), np.uint8)
        thresh_clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        thresh_clean = cv2.morphologyEx(thresh_clean, cv2.MORPH_CLOSE, kernel)
        
        # Detectar blobs usando SimpleBlobDetector
        # Configurar los parámetros del detector
        params = cv2.SimpleBlobDetector_Params()
        
        # Filtro por área
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 10000
        
        # Filtro por circularidad
        params.filterByCircularity = True
        params.minCircularity = 0.1
        
        # Filtro por convexidad
        params.filterByConvexity = True
        params.minConvexity = 0.5
        
        # Filtro por inercia (forma alargada vs. circular)
        params.filterByInertia = True
        params.minInertiaRatio = 0.1
        
        # Crear el detector de blobs
        detector = cv2.SimpleBlobDetector_create(params)
        
        # Detectar blobs
        keypoints = detector.detect(thresh_clean)
        
        # Dibujar los blobs detectados como círculos
        img_with_keypoints = cv2.drawKeypoints(
            result, 
            keypoints, 
            np.array([]), 
            (0, 0, 255), 
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
        )
        
        # Usar el método de contornos para encontrar centroides
        # Encontrar contornos en la imagen binaria
        contornos, _ = cv2.findContours(
            thresh_clean, 
            cv2.RETR_EXTERNAL, 
            cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Número de blobs encontrados por contornos
        num_blobs = len(contornos)
        
        # Dibujar contornos y centroides
        for i, contorno in enumerate(contornos):
            # Calcular el área para filtrar contornos muy pequeños
            area = cv2.contourArea(contorno)
            if area < 100:  # Ignorar áreas muy pequeñas
                continue
                
            # Dibujar el contorno
            cv2.drawContours(img_with_keypoints, [contorno], -1, (0, 255, 0), 2)
            
            # Calcular momentos
            M = cv2.moments(contorno)
            
            # Calcular el centroide solo si el área no es cero
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Dibujar el centro como un círculo
                cv2.circle(img_with_keypoints, (cx, cy), 5, (255, 0, 0), -1)
                
                # Añadir etiqueta con el número de blob
                cv2.putText(
                    img_with_keypoints, 
                    f"Blob {i+1}", 
                    (cx - 25, cy - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    (255, 0, 0), 
                    2
                )
        
        # Añadir información total de blobs en la esquina superior
        cv2.putText(
            img_with_keypoints,
            f"Total: {num_blobs} blobs",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            2
        )
        
        return img_with_keypoints
    except Exception as e:
        print(f"Error al detectar blobs: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

def aplicar_canny_simple(img_path):
    """
    Implementa el algoritmo de Canny de forma simple y directa.
    Muestra la imagen original, la imagen con desenfoque gaussiano y la imagen con detección de bordes.
    
    Args:
        img_path (str): Ruta a la imagen de origen
        
    Returns:
        numpy.ndarray: Imagen procesada con Canny o None si hay error
    """
    try:
        print(f"Iniciando aplicar_canny_simple en: {img_path}")
        # Verificar que el archivo existe
        if not os.path.exists(img_path):
            print(f"¡ERROR! El archivo no existe: {img_path}")
            return None
            
        # Cargar la imagen
        img = cv2.imread(img_path)
        if img is None:
            print(f"¡ERROR! No se pudo cargar la imagen: {img_path}")
            return None
            
        print(f"Imagen cargada correctamente, tamaño: {img.shape}")
        
        # Crear una copia para dibujar los resultados
        result = img.copy()
        
        # 1. Convertir a escala de grises
        try:
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            print("Conversión a escala de grises exitosa")
        except Exception as e:
            print(f"Error en conversión a escala de grises: {str(e)}")
            return None
        
        # 2. Aplicar suavizado Gaussiano para reducir ruido
        try:
            img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
            print("Aplicación de desenfoque gaussiano exitosa")
        except Exception as e:
            print(f"Error en aplicación de desenfoque: {str(e)}")
            return None
        
        # 3. Aplicar algoritmo de Canny para detección de bordes
        try:
            edges = cv2.Canny(img_blur, 50, 50)
            print("Aplicación de algoritmo Canny exitosa")
        except Exception as e:
            print(f"Error en aplicación de Canny: {str(e)}")
            return None
        
        # 4. Encontrar contornos en la imagen con bordes
        try:
            contornos, _ = cv2.findContours(
                edges.copy(),
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )
            print(f"Encontrados {len(contornos)} contornos")
        except Exception as e:
            print(f"Error al encontrar contornos: {str(e)}")
            return None
        
        # 5. Crear las tres imágenes para la visualización
        try:
            # Convertir imágenes a color para mostrarlas juntas
            img_blur_color = cv2.cvtColor(img_blur, cv2.COLOR_GRAY2BGR)
            edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            
            # Dibujar contornos en verde sobre la imagen original
            img_contours = result.copy()
            cv2.drawContours(
                image=img_contours,
                contours=contornos,
                contourIdx=-1,
                color=(0, 255, 0),
                thickness=2,
                lineType=cv2.LINE_AA
            )
            
            # Combinar las tres imágenes para la visualización
            height, width = img.shape[:2]
            print(f"Dimensiones de la imagen: {width}x{height}")
            
            # Crear un lienzo para las tres imágenes
            combined = np.zeros((height, width * 3, 3), dtype=np.uint8)
            
            # Colocar las tres imágenes lado a lado
            combined[:, 0:width] = img_blur_color
            combined[:, width:width*2] = edges_color
            combined[:, width*2:width*3] = img_contours
            
            # Agregar textos explicativos
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(combined, "Desenfocada", (10, 30), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(combined, "Bordes Canny", (width + 10, 30), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(combined, "Contornos", (width * 2 + 10, 30), font, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            
            # Agregar líneas divisorias
            cv2.line(combined, (width, 0), (width, height), (255, 255, 255), 2)
            cv2.line(combined, (width * 2, 0), (width * 2, height), (255, 255, 255), 2)
            
            # Información adicional
            num_contornos = len(contornos)
            cv2.putText(
                combined, 
                f"Umbral: 50-50, Contornos: {num_contornos}", 
                (width + 10, height - 20), 
                font, 
                0.6, 
                (255, 255, 255), 
                1, 
                cv2.LINE_AA
            )
            
            print("Imagen combinada creada exitosamente")
            return combined
        except Exception as e:
            print(f"Error al crear la imagen combinada: {str(e)}")
            import traceback
            traceback.print_exc()
            return None
    except Exception as e:
        print(f"Error general en aplicar_canny_simple: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

@app.route('/api/transform', methods=['POST'])
def transform_image():
    data = request.json
    filename = data.get('filename')
    transform_type = data.get('transform_type')

    print(f"Procesando transformación: {transform_type} para archivo: {filename}")

    if not filename or not transform_type:
        return jsonify({'error': 'Faltan parámetros'}), 400

    img_path = None
    # Buscar la imagen en todas las categorías
    for root, dirs, files in os.walk(PUBLIC_DIR):
        if filename in files:
            img_path = os.path.join(root, filename)
            print(f"Imagen encontrada en: {img_path}")
            break

    if not img_path or not os.path.exists(img_path):
        print(f"¡ERROR! Imagen no encontrada: {filename}")
        return jsonify({'error': 'Imagen no encontrada'}), 404

    try:
        if transform_type == 'color':
            res = detectar_colores(img_path)
            if res is None:
                return jsonify({'error': 'No se pudo procesar la imagen'}), 500

            _, buffer = cv2.imencode('.png', res)
            return send_file(
                io.BytesIO(buffer.tobytes()),
                mimetype='image/png',
                as_attachment=False,
                download_name='color.png'
            )
        elif transform_type == 'edges':
            print(f"Aplicando transformación de bordes a {img_path}")
            res = aplicar_canny(img_path)
            if res is None:
                print("Error: aplicar_canny devolvió None")
                return jsonify({'error': 'No se pudo procesar la imagen'}), 500

            _, buffer = cv2.imencode('.png', res)
            return send_file(
                io.BytesIO(buffer.tobytes()),
                mimetype='image/png',
                as_attachment=False,
                download_name='edges.png'
            )
        elif transform_type == 'canny':
            print(f"Aplicando transformación Canny simple a {img_path}")
            res = aplicar_canny_simple(img_path)
            if res is None:
                print("Error: aplicar_canny_simple devolvió None")
                return jsonify({'error': 'No se pudo procesar la imagen'}), 500

            print(f"Imagen procesada exitosamente con tamaño: {res.shape}")
            _, buffer = cv2.imencode('.png', res)
            return send_file(
                io.BytesIO(buffer.tobytes()),
                mimetype='image/png',
                as_attachment=False,
                download_name='canny.png'
            )
        elif transform_type == 'shapes':
            res = detectar_formas(img_path)
            if res is None:
                return jsonify({'error': 'No se pudo procesar la imagen'}), 500

            _, buffer = cv2.imencode('.png', res)
            return send_file(
                io.BytesIO(buffer.tobytes()),
                mimetype='image/png',
                as_attachment=False,
                download_name='shapes.png'
            )
        elif transform_type == 'blobs':
            res = detectar_blobs(img_path)
            if res is None:
                return jsonify({'error': 'No se pudo procesar la imagen'}), 500

            _, buffer = cv2.imencode('.png', res)
            return send_file(
                io.BytesIO(buffer.tobytes()),
                mimetype='image/png',
                as_attachment=False,
                download_name='blobs.png'
            )
        else:
            print(f"Transformación no soportada: {transform_type}")
            return jsonify({'error': 'Transformación no soportada'}), 400
    except Exception as e:
        print(f"ERROR al procesar imagen {filename} con transformación {transform_type}: {str(e)}")
        import traceback
        traceback.print_exc()
        return jsonify({'error': f'Error interno: {str(e)}'}), 500

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True)
