from setuptools import setup

package_name = 'task_action'

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
    maintainer='akash',
    maintainer_email='akash.sambhus@rwth-aachen.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['action_server = task_action.action_server:main',
            'action_client = task_action.action_client:main',
        ],
    },
)
