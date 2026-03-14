from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='victor',
    maintainer_email='victor@todo.com',
    description='Top-level launch and pick-and-place pipeline for humanoid arm',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pick_place_node = humanoid_bringup.pick_place_node:main',
        ],
    },
)
