from setuptools import find_packages, setup
from glob import glob

package_name = 'robinz_vehicle_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/csv', glob('csv/*.csv')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.config')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nontanan',
    maintainer_email='nontanan@gensurv.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'initialpose = robinz_vehicle_launch.auto_initialpose:main',
        ],
    },
)
