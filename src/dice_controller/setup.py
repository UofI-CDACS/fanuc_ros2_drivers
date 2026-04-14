import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dice_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CS4554 Student',
    maintainer_email='student@uidaho.edu',
    description='Dice pick-and-inspect nodes for FANUC robot',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'camera_node  = dice_controller.camera_node:main',
            'master_node  = dice_controller.master_node:main',
        ],
    },
)
