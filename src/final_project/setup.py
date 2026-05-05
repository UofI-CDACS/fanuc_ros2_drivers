import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'final_project'

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
    description='CS4554 Final — dual-robot sequential dice inspection',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'camera_server  = final_project.camera_server:main',
            'robot1_master  = final_project.robot1_master:main',
            'robot2_master  = final_project.robot2_master:main',
            'modbus_server  = final_project.modbus_server:main',
            'bunsen_debug   = final_project.bunsen_debug:main',
        ],
    },
)
