
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'zenoh_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer_email='anandchowdary2005@gmail.com',
    description='Zenoh bridge for cloud communication',
    license='MIT',
    entry_points={
        'console_scripts': [
            'zenoh_bridge_node = zenoh_bridge.zenoh_bridge_node:main',
        ],
    },
)