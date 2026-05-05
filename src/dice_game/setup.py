import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dice_game'

setup(
    name=package_name,
    version='0.1.0',
    packages=['dice_game', 'dependencies'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='todo@uidaho.edu',
    description='Dice game nodes for two-robot sequential pip inspection',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'camera_server    = dice_game.camera_server_node:main',
            'robot1_ctrl      = dice_game.robot1_controller:main',
            'robot2_ctrl      = dice_game.robot2_controller:main',
        ],
    },
)
