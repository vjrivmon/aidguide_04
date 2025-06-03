from setuptools import setup
"""Módulo <module>.

Este módulo proporciona funcionalidades para el proyecto AidGuide 04.
"""
import os

package_name = 'aidguide_04_capture_image'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the clasificadores directory and its contents
        (os.path.join('share', package_name, 'clasificadores'), ['aidguide_04_capture_image/clasificadores/haarcascade_car.xml']),
        (os.path.join('share', package_name, 'clasificadores'), ['aidguide_04_capture_image/clasificadores/haarcascade_frontalface_default.xml']),
        (os.path.join('share', package_name, 'clasificadores'), ['aidguide_04_capture_image/clasificadores/cascade_pesh_lbp14.xml']),
        (os.path.join('share', package_name, 'clasificadores'), ['aidguide_04_capture_image/clasificadores/cascade_stop_3_15.xml']),
        (os.path.join('share', package_name, 'clasificadores'), ['aidguide_04_capture_image/clasificadores/cascade_zepr_lbp13.xml']),
        (os.path.join('share', package_name, 'clasificadores'), ['aidguide_04_capture_image/clasificadores/TrafficLight_HAAR_16Stages.xml']),
        ],
        
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugo',
    maintainer_email='hugo@example.com',
    description='ROS2 package for capturing images and detecting cars',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_image = aidguide_04_capture_image.capturar:main',
        ],
    },
)