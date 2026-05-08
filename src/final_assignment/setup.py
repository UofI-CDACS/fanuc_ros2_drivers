import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'final_assignment'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Habermann',
    maintainer_email='todo@todo.com',
    description='Two-robot dice coordination — final assignment',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_client = final_assignment.camera_client_node:main',
            'hsv_tuner = final_assignment.hsv_tuner_node:main',
        ],
    },
)
