from setuptools import setup
from glob import glob
import os

package_name = 'robot_sim_safety'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duc',
    maintainer_email='duc@todo.todo',
    description='Simulation safety provider for battery, water, and emergency stop in Gazebo.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_safety_node = robot_sim_safety.sim_safety_node:main',
        ],
    },
)

