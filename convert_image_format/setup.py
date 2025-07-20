from setuptools import find_packages, setup

package_name = 'convert_image_format'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jian Chen',
    maintainer_email='j.chen3.24@abdn.ac.uk',
    description='Convert rgba8 to rgb8',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'convert_image_format = convert_image_format.convert_image_format:main',
        ],
    },
)
