from setuptools import setup
import os
from glob import glob

package_name = 'aidguide_04_deep_learning'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir archivos de lanzamiento
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Incluir modelos y otros recursos
        (os.path.join('share', package_name, 'resource'), glob('resource/*.*')), # Asume que el modelo estará en resource/
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vicente',
    maintainer_email='vicente.carrasco@gmail.com',
    description='Paquete ROS2 para la detección de frutas usando un modelo YOLO entrenado con TensorFlow Lite.',
    license='Apache License 2.0', # O la licencia que prefieras
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fruit_detector = aidguide_04_deep_learning.fruit_detector_node:main',
        ],
    },
) 