from setuptools import setup
import os

package_name = 'task'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/move_robot.launch.py']),(os.path.join('share', package_name, 'launch'), ['launch/execute_task.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akash',
    maintainer_email='akash.sambhus@rwth-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['controller_node=task.controller_node:main','robot_node=task.robot_node:main'
        ],
    },
)
