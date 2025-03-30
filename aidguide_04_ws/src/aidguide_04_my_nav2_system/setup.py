from setuptools import setup
import os
from glob import glob

package_name = 'aidguide_04_my_nav2_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mimivladeva',
    maintainer_email='mvladev@epsg.upv.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'punto_inicial = aidguide_04_my_nav2_system.punto_inicial:main'
        ],
    },
)

