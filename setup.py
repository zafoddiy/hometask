import os
from setuptools import find_packages, setup
from glob import glob


package_name = 'ias0220_231899'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elias Sepp',
    maintainer_email='seppelias1@gmail.com',
    description='Differential robot package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'walker = ias0220_231899.random_walker:main',
                'position_calculator = ias0220_231899.odometer:main',
        ],
    },
)
