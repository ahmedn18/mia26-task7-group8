import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mia_task7'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='masa',
    maintainer_email='masamostafa2017@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_x_server = mia_task7.move_x_server:main',
            'move_x_client = mia_task7.move_x_client:main',
            'yaw_server = mia_task7.yaw_server:main',
            'move_yaw_client = mia_task7.move_yaw_client:main',
        ],
    },
)
