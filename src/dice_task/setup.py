import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dice_task'

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
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ben-kopf',
    maintainer_email='todo@example.com',
    description='Async state machine for dice inspection with FANUC CRX-10 and OnRobot gripper',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'dice_task_node = dice_task.dice_task_node:main',
        ],
    },
)
