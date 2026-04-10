import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dice_task'

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
    maintainer='TODO',
    maintainer_email='todo@uidaho.edu',
    description='Dice pick-and-count task for FANUC CRX10 with MindVision camera',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'mv_camera_node = dice_task.mv_camera_node:main',
            'dice_roller     = dice_task.dice_roller:main',
        ],
    },
)
