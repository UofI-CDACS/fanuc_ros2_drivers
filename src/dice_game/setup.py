from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dice_game'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kate-bouse',
    maintainer_email='todo@todo.com',
    description='Dice pip sequential counting game using two FANUC robots.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_server = dice_game.camera_server:main',
            'dj_control    = dice_game.dj_control:main',
            'bill_control  = dice_game.bill_control:main',
        ],
    },
)
