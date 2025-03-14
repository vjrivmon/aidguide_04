from setuptools import setup
import os
from glob import glob

package_name = 'aidguide_04'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vicente Rivas Monferrer',
    maintainer_email='vjrivmon@epsg.upv.es',
    description='Sistema de guiado asistido para robots móviles',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = aidguide_04.main_node:main',
            'navigation_node = aidguide_04.navigation_node:main',
            'safety_node = aidguide_04.safety_node:main'
        ],
    },
) 