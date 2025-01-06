from setuptools import find_packages, setup

package_name = 'robinz_stereo_camera'

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
    maintainer_email='nontanan@gensurv.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = robinz_stereo_camera.camera:main',
            'stereo_camera_node = robinz_stereo_camera.stereo_camera_node:main',
            'stereo_depth_estimator = robinz_stereo_camera.stereo_depth_estimation:main',
        ],
    },
)
