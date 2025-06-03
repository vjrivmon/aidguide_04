#!/bin/bash

# Este script crea los symlinks necesarios para que ROS2 encuentre los ejecutables
# en la ubicación correcta después de la instalación

# Directorio donde se instala el ejecutable por defecto
BIN_DIR="/home/irene/aidguide_04/aidguide_04_ws/install/aidguide_04_deep_learning/bin"

# Directorio donde ROS2 busca los ejecutables
LIB_DIR="/home/irene/aidguide_04/aidguide_04_ws/install/aidguide_04_deep_learning/lib/aidguide_04_deep_learning"

# Crear el directorio si no existe
mkdir -p "$LIB_DIR"

# Crear symlink para el ejecutable fruit_detector
if [ -f "$BIN_DIR/fruit_detector" ]; then
    ln -sf "$BIN_DIR/fruit_detector" "$LIB_DIR/fruit_detector"
    echo "Symlink creado para fruit_detector"
else
    echo "ERROR: No se encontró el ejecutable fruit_detector en $BIN_DIR"
    exit 1
fi

echo "Symlinks creados correctamente" 