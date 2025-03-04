from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb3_multi_env_spawner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mengo',
    maintainer_email='riccardo.mengozzi3@studio.unibo.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_spawner = tb3_multi_env_spawner.robot_spawner:main',
            'reset_environment = tb3_multi_env_spawner.reset_environment:main',
            'envs_properties_publisher = tb3_multi_env_spawner.envs_properties_publisher:main',
        ],
    },
)
