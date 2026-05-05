import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'dual_fanuc'

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
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Dual-robot FANUC dice assignment',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'robot1         = dual_fanuc.robot1:main',
            'robot2         = dual_fanuc.robot2:main',
            'mv_camera_node = dual_fanuc.mv_camera_node:main',
            'rotate1        = dual_fanuc.robot1:main_rotate',
            'rotate2        = dual_fanuc.robot2:main_rotate',
        ],
    },
)
