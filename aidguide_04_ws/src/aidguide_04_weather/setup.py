from setuptools import setup
import os
from glob import glob

package_name = 'aidguide_04_weather'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irene',
    maintainer_email='imedgar@epsg.upv.es',
    description='Paquete para recibir información del clima y advertir sobre lluvia o calor extremo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'weather_monitor = aidguide_04_weather.weather_monitor:main',
        ],
    },
) 