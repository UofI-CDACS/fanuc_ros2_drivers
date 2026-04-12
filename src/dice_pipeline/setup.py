import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dice_pipeline'

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
    maintainer='TODO',
    maintainer_email='todo@example.com',
    description='Camera and master control nodes for dice pip counting pipeline',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'camera_node = dice_pipeline.camera_node:main',
            'master_node = dice_pipeline.master_node:main',
        ],
    },
)
