from setuptools import setup
from glob import glob
import os

package_name = 'rl_training'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='myc',
    maintainer_email='myc@todo.todo',
    description='Global-state RL weight generator for fuzzy exploration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_global_state_node = rl_training.rl_global_state_node:main',
            'rl_weight_node = rl_training.rl_weight_node:main',
            'rl_fixed_weight_node = rl_training.fixed_weight_node:main',
            'rl_infer_mappo = rl_training.infer_mappo:main',
        ],
    },
)
