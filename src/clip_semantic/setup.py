from setuptools import find_packages, setup
import os

package_name = 'clip_semantic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
     ['launch/clip_semantic_launch.py']),
        (os.path.join('share', package_name, 'config'),
     [
         'config/static_semantic_scores.json',
         'config/static_semantic_grid_model.npz',
         'config/static_semantic_grid_real.npz',
         'config/static_semantic_grid_maze40.npz',
     ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huishao',
    maintainer_email='huishao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clip_semantic_node = clip_semantic.clip_semantic_node:main',
            'semantic_mapper = clip_semantic.semantic_map:main',
            'oracle_semantic_mapper = clip_semantic.oracle_semantic_mapper:main',
        ],
    },
)
