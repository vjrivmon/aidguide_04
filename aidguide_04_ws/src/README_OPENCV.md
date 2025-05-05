# Procesamiento de Imágenes con OpenCV - Detector de Contornos

Este proyecto implementa un sistema para procesar imágenes utilizando OpenCV, específicamente para la detección de contornos. El sistema procesa imágenes de diferentes categorías (señales de tráfico, personas, paradas de autobús, etc.) y las integra con una interfaz web.

## Características

- Detección de contornos en imágenes utilizando OpenCV
- Clasificación de imágenes en categorías
- Visualización de imágenes procesadas
- Integración con interfaz web React/Next.js

## Estructura de Archivos

- `image_processor.py`: Procesa las imágenes y detecta contornos
- `view_processed_images.py`: Visualizador para ver las imágenes originales y procesadas
- `integrate_processed_images.py`: Integra las imágenes procesadas con la interfaz web
- `process_all_images.py`: Script principal que ejecuta toda la cadena de procesamiento
- `app/components/ImageGrid.tsx`: Componente React para mostrar imágenes en la interfaz web

## Requisitos

- Python 3.6 o superior
- OpenCV (cv2)
- NumPy
- React/Next.js (para la interfaz web)

## Instalación de Dependencias

Para instalar las dependencias necesarias, ejecute:

```bash
pip install opencv-python numpy
```

## Uso

### Procesamiento de Imágenes

Para iniciar todo el proceso de una vez:

```bash
python process_all_images.py
```

Este script ejecutará automáticamente:
1. El procesamiento de imágenes para detectar contornos
2. La integración con la interfaz web

### Pasos Individuales

Si prefieres ejecutar los pasos individualmente:

1. Procesar imágenes:
   ```bash
   python image_processor.py
   ```

2. Integrar con la interfaz web:
   ```bash
   python integrate_processed_images.py
   ```

3. Visualizar las imágenes procesadas:
   ```bash
   python view_processed_images.py
   ```

## Funcionamiento

### Detección de Contornos

El proceso de detección de contornos sigue estos pasos:

1. Transformar la imagen a escala de grises.
2. Aplicar umbralización binaria (threshold).
3. Buscar los contornos de la imagen.
4. Dibujar los contornos en la imagen original.

### Clasificación de Imágenes

Las imágenes se clasifican en las siguientes categorías:
- Señales de tráfico
- Personas
- Paradas de autobús
- Pasos de peatones
- Obras
- Calles cortadas

### Visualización

El visualizador permite:
- Ver imágenes originales y procesadas lado a lado
- Navegar entre imágenes con teclas (A/D o flechas)
- Salir con la tecla ESC

### Integración con la Interfaz Web

Las imágenes procesadas se integran con la interfaz web React/Next.js y se pueden visualizar seleccionando una categoría en la sección "Imágenes captadas por el robot".

## Personalización

### Ajustar Umbralización

Para ajustar el umbral de detección, modifica el valor `155` en la función `cv2.threshold`:

```python
ret, umbral = cv2.threshold(img_gray, 155, 255, cv2.THRESH_BINARY)
```

### Cambiar Color de Contornos

Para cambiar el color de los contornos, modifica los valores BGR en la función `cv2.drawContours`:

```python
cv2.drawContours(img_contornos, contornos, -1, (0, 165, 255), 3)
```

## Resolución de Problemas

### Imágenes No Detectadas Correctamente

Si los contornos no se detectan correctamente:
- Prueba con diferentes valores de umbral
- Utiliza otros métodos de preprocesamiento, como filtros de suavizado (blur) o detección de bordes (Canny)

### Errores de Integración Web

Si las imágenes no aparecen en la interfaz web:
- Verifica que los directorios de imágenes existen y tienen permisos correctos
- Comprueba que el servidor web está en ejecución
- Revisa la consola del navegador para errores JavaScript

## Referencias

- [Documentación oficial de OpenCV](https://docs.opencv.org/)
- [Tutorial de contornos en OpenCV](https://docs.opencv.org/master/d4/d73/tutorial_py_contours_begin.html)
- [Tutorial de detección de contornos](https://docs.opencv.org/4.5.5/df/d0d/tutorial_find_contours.html) 