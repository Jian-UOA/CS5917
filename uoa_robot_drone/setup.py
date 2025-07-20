import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'uoa_robot_drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools', 'djitellopy'],
    zip_safe=True,
    maintainer='Jian Chen',
    maintainer_email='j.chen3.24@abdn.ac.uk',
    description='Using robots and drones to delivery, copyright belongs to University of Aberdeen',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_delivery = uoa_robot_drone.auto_delivery:main',
            'robot_position_listener = uoa_robot_drone.robot_position_listener:main',
        ],
    },
)
