import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'action_servers'

setup(
    name=package_name,
    version='0.0.1',
    packages=['action_servers','dependencies'],
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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cart_pose_server = action_servers.cart_pose_server:main'
        ],
    },
)

