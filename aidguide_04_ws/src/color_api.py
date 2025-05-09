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
            
        # Convertir a escala de grises
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Aplicar suavizado Gaussiano para reducir ruido
        img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
        
        # Aplicar algoritmo de Canny para detección de bordes
        edges = cv2.Canny(img_blur, 50, 150)
        
        # Dilatación para mejorar la visibilidad de los bordes
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=1)
        
        # Convertir bordes a BGR para poder colorearlos
        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        
        # Colorear los bordes (naranja)
        edges_color[np.where((edges_color == [255, 255, 255]).all(axis=2))] = [0, 165, 255]
        
        # Combinar con la imagen original para mejor visualización
        result = cv2.addWeighted(img, 0.7, edges_color, 0.9, 0)
        
        return result
    except Exception as e:
        print(f"Error al aplicar Canny: {str(e)}")
        return None

@app.route('/api/transform', methods=['POST'])
def transform_image():
    data = request.json
    filename = data.get('filename')
    transform_type = data.get('transform_type')

    if not filename or not transform_type:
        return jsonify({'error': 'Faltan parámetros'}), 400

    img_path = None
    # Buscar la imagen en todas las categorías
    for root, dirs, files in os.walk(PUBLIC_DIR):
        if filename in files:
            img_path = os.path.join(root, filename)
            break

    if not img_path or not os.path.exists(img_path):
        return jsonify({'error': 'Imagen no encontrada'}), 404

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
        res = aplicar_canny(img_path)
        if res is None:
            return jsonify({'error': 'No se pudo procesar la imagen'}), 500

        _, buffer = cv2.imencode('.png', res)
        return send_file(
            io.BytesIO(buffer.tobytes()),
            mimetype='image/png',
            as_attachment=False,
            download_name='edges.png'
        )
    else:
        return jsonify({'error': 'Transformación no soportada'}), 400

if __name__ == "__main__":
    app.run(debug=True)
