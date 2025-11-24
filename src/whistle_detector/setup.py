from setuptools import setup
import os
from glob import glob

package_name = 'whistle_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anand',
    maintainer_email='anand@example.com',
    description='Whistle detection node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'whistle_detector_node = whistle_detector.whistle_detector_node:main',
            'autonomous_responder = whistle_detector.autonomous_responder:main',
            'sound_source_visualizer = whistle_detector.sound_source_visualizer:main',
        ],
    },
)
