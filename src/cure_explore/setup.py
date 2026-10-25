from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cure_explore'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install package marker (required for ament)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # Install RViz config
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
        # Install documentation
        (os.path.join('share', package_name, 'doc'),
            glob(os.path.join('doc', '*.md'))),
        # Install map directory (all subdirectories and files)
        (os.path.join('share', package_name, 'map'),
            glob('map/*.*', recursive=False)),
    ] + [
        # Install each map subdirectory
        (os.path.join('share', package_name, 'map', d),
         glob(os.path.join('map', d, '*')))
        for d in os.listdir('map') if os.path.isdir(os.path.join('map', d))
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Song Yuyang',
    maintainer_email='s70735526@mail.163.com',
    description='Multi-Robot Exploration with CURE algorithm for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_position_publisher = cure_explore.robot_position_publisher:main',
            'map_tf_broadcaster = cure_explore.map_tf_broadcaster:main',
            'pose_config_server = cure_explore.pose_config_server:main',
            'dynamic_tf_broadcaster = cure_explore.dynamic_tf_broadcaster:main',
        ],
    },
)
