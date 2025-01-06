from setuptools import find_packages, setup

package_name = 'grid_point_node'

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
    maintainer='nontanan',
    maintainer_email='nontanansommat123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grid_point_node = grid_point_node.grid_point_node:main',
            'map_republisher = grid_point_node.map_republisher:main',
            'path_planner = grid_point_node.path_planner:main',
        ],
    },
)
