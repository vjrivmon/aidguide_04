from setuptools import setup
import os
from glob import glob

package_name = 'aidguide_04_robot_monitoring'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irene',
    maintainer_email='imedgar@epsg.upv.es',
    description='Monitorización y Diagnóstico del Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'battery_monitor = aidguide_04_robot_monitoring.battery_monitor:main',
            'hardware_monitor = aidguide_04_robot_monitoring.hardware_monitor:main',
            'temperature_monitor = aidguide_04_robot_monitoring.temperature_monitor:main',
            'log_monitor = aidguide_04_robot_monitoring.log_monitor:main',
            'monitoring_dashboard = aidguide_04_robot_monitoring.monitoring_dashboard:main',
        ],
    },
) 