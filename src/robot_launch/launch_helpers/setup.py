import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'launch_helpers'

setup(
    name=package_name,
    version='1.5.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/eef_control_registry.yaml')),
        (os.path.join('share', package_name, 'config/ros2_control/templates'),
         glob('config/ros2_control/templates/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='TODO@email.com',
    description='Shared Python library for robot launch',
    license='BSD-3',
)
