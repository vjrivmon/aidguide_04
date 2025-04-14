from setuptools import setup
import os
from glob import glob

package_name = 'aidguide_04_nav'

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
    description='TODO: Movimiento del robot con el paquete de navegación (cmd_vel)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aidguide_navigation = aidguide_04_nav.aidguide_04_nav:main',
        ],
    },
)
