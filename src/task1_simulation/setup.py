from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task1_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),

    ('share/' + package_name,
        ['package.xml']),

    (os.path.join('share', package_name, 'models', 'moving_wall'),
    glob('models/moving_wall/*')),
    
    (os.path.join('share', package_name, 'models', 'turtlebot3_burger_3d_lidar'),
    glob('models/turtlebot3_burger_3d_lidar/*')),

    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.py')),

    (os.path.join('share', package_name, 'worlds'),
        glob('worlds/*.world')),

    (os.path.join('share', package_name, 'models', '3d_lidar'),
        glob('models/3d_lidar/*')),
],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='anubhav',
    maintainer_email='anubhav96022@gmail.com',

    description='Task 1 simulation for Pace Robotics assignment',
    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
        ],
    },
)