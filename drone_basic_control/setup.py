from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_basic_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='anna.astolfi@nttdata.com',
    description='Basilar nodes to command drone via PX4 Autopilot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hb_node = drone_basic_control.heartbeat_node:main',
            'arming_node = drone_basic_control.arming_node:main',
            'takeoff_node = drone_basic_control.takeoff_node:main',
            'landing_node = drone_basic_control.landing_node:main',
        ],
    },
)
