from setuptools import setup
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