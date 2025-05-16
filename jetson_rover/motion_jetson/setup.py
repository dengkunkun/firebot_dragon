from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'motion_jetson'

setup(
    name=package_name,  # Important: Use underscores, not hyphens
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cat',
    maintainer_email='cat@todo.todo',
    description='Motion control package for jetson rover',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_jetson = motion_jetson.motion_jetson:main'
        ],
    },
)