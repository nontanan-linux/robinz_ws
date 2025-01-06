from setuptools import setup

package_name = 'py_purepursuit_gs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gs',
    maintainer_email='gs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'purepursuit = py_purepursuit_gs.gs_purepursuit_ros2:main',
            'bb_purepursuit = py_purepursuit_gs.bb_purepursuit_ros2:main',
            'bb_docking = py_purepursuit_gs.bb_line_docking2:main',
        ],
    },
)
