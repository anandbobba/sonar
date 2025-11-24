
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'alert_manager'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anand',
    maintainer_email='anandchowdary2005@gmail.com',
    description='Alert management and cloud forwarding',
    license='MIT',
    entry_points={
        'console_scripts': [
            'alert_manager_node = alert_manager.alert_manager_node:main',
        ],
    },
)