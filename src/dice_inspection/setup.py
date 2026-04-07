import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dice_inspection'

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
    maintainer='student',
    maintainer_email='todo@example.com',
    description='FANUC dice pick-and-inspect ROS2 nodes',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_node = dice_inspection.camera_node:main',
            'master_node = dice_inspection.master_node:main',
            'test_robot_motion = dice_inspection.test_robot_motion:main',
            'claude_assignment = dice_inspection.claude_assignment:main',
        ],
    },
)
