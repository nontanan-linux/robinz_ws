from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robinz_vehicle_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Automatically find packages
    data_files=[
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include rviz files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Description of the robinz_vehicle_launch package',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'python_node = robinz_vehicle_launch.python_node:main',
            'auto_initialpose = robinz_vehicle_launch.auto_initialpose:main',
        ],
    },
)
