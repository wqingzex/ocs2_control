import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='1.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config/nav2'), glob('config/nav2/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Huang Zhenbiao',
    maintainer_email='biao876990970@hotmail.com',
    description='Nav2 launch and map / parameter lookup for robots',
    license='BSD-3',
)
