import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'formation_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joel McCandless',
    maintainer_email='mail@joelmccandless.com',
    description='Multi-robot formation control for ROSMASTER-X3 hardware',
    license='MIT',
    entry_points={
        'console_scripts': [
            'formation_hw = formation_control.formation_hw_node:main',
        ],
    },
)
