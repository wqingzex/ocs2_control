import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_visualize'

setup(
    name=package_name,
    version='1.5.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config/rviz'), glob('config/rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Huang Zhenbiao',
    maintainer_email='biao876990970@hotmail.com',
    description='RViz model viewing launch files',
    license='BSD-3',
)
