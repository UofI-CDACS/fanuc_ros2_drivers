import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'srv_services'

setup(
    name=package_name,
    version='0.1.1',
    packages=['srv_services','dependencies'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kris Olds',
    maintainer_email='kolds@uidaho.edu',
    description='ROS2 implementation for Fanuc CRX10',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mount_position = srv_services.mount_position:main',
            'set_speed = srv_services.set_speed:main'
        ],
    },
)
