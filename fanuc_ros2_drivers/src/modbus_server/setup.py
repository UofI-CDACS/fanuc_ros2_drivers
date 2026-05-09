import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'modbus_server'

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
    maintainer='ben-kopf',
    maintainer_email='kopf3990@vandals.uidaho.edu',
    description='Modbus TCP server node holding shared robot state registers and coils',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'modbus_server_node = modbus_server.modbus_server_node:main',
        ],
    },
)
