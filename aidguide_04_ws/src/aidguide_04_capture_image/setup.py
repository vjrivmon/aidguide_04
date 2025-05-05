from setuptools import setup
import os
from glob import glob

package_name = 'aidguide_04_capture_image'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Incluir scripts ejecutables
        (os.path.join('lib', package_name), glob('aidguide_04_capture_image/*.py')),
        # Incluir la carpeta clasificadores
        (os.path.join('share', package_name, 'clasificadores'), glob('aidguide_04_capture_image/clasificadores/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugo',
    maintainer_email='your_email@example.com',
    description='Paquete para captura de imágenes y detección de personas',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_image = aidguide_04_capture_image.capturar:main',
        ],
    },
)