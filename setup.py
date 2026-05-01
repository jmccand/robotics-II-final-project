from setuptools import find_packages, setup

package_name = 'formation_control'

setup(
    name=package_name,
    version='0.1.0',
    # Finds: formation_control, behavior, virtual_structure, leader_follower
    packages=find_packages(exclude=['test']),
    # Root-level simulation modules installed alongside the packages
    py_modules=[
        'dynamics',
        'formation',
        'formation_controller',
        'path',
        'obstacles',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hardware.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'formation_node = formation_control.formation_node:main',
        ],
    },
)
