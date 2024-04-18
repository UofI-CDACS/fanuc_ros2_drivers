import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'action_servers'

setup(
    name=package_name,
    version='0.1.1',
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
            'cart_pose_server = action_servers.cart_pose_server:main',
            'convey_server = action_servers.convey_server:main',
            'joint_pose_server = action_servers.joint_pose_server:main',
            'onrobot_server = action_servers.onrobot_server:main',
            'schunk_server = action_servers.schunk_server:main',
            'single_joint_server = action_servers.single_joint_server:main',
        ],
    },
)

