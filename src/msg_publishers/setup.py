import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'msg_publishers'

setup(
    name=package_name,
    version='0.0.1',
    packages=['msg_publishers','dependencies'],
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
            'current_cart = msg_publishers.current_cart:main',
            'current_grip = msg_publishers.current_grip:main',
            'current_joint = msg_publishers.current_joint:main',
            'move_check = msg_publishers.move_check:main',
            'prox_check = msg_publishers.prox_check:main',
            'speed_check = msg_publishers.speed_check:main',
        ],
    },
)
